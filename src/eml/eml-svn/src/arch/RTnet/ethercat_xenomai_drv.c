// $Id:$
//===========================================================================
//	This file is part of "EtherCAT Master Library".
//	Copyright (C) 2005 FMTC vzw, Diamant Building, A. Reyerslaan 80,
//	B-1030 Brussels, Belgium.
//
//	EtherCAT Master Library is free software; you can redistribute it
//	and/or modify it under the terms of the GNU General Public License
//	as published by the Free Software Foundation; either version 2 or
//	(at your option) any later version.
//
//	EtherCAT Master Code is distributed in the hope that it will be
//	useful, but WITHOUT ANY WARRANTY; without even the implied
//	warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//	PURPOSE. See the GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with the EtherCAT Master Library; if not, write to the Free
//	Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
//	02111-1307 USA.
//
//	EtherCAT, the EtherCAT trade name and logo are the intellectual
//	property of, and protected by Beckhoff. You can use "EtherCAT
//	Master Library" for creating and/or selling or otherwise
//	distributing an EtherCAT network master under the terms of the
//	EtherCAT Master License.
//
//	You should have received a copy of the EtherCAT Master License
//	along with the EtherCAT Master Library; if not, write to Beckhoff
//	Automation GmbH, Eiserstrasse 5, D-33415 Verl, Germany.
//===========================================================================


#include <string.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>	
#include <sys/ioctl.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <rtnet.h>

#include <ethercat/netif.h>
#include <ethercat/ethercat_log.h>
#include <ethercat/ethercat_xenomai_drv.h>

// Maximum tries to send and receive a message
#define MAX_TRIES_TX 10
// Timeout for receiving and transmitting messages is 100 us
#define TIMEOUT_NSEC 1000 * 100
// Maximum times the master may retry to create or close a socket
#define MAX_TRIES_SOCKET 10
// Ethernet Header is 14 bytes: 2 * MAC ADDRESS + 2 bytes for the type
#define HEADER_SIZE 14
#define MAX_ETH_DATA 1500

int set_socket_timeout(struct netif* ni, int64_t timeout) {
	return ioctl(ni->socket_private,RTNET_RTIOC_TIMEOUT,&timeout);
}

int init_socket(const char* interface) {
	int sock;
	int tries = 0;
	struct sockaddr_ll addr;
	struct ifreq ifr;

	while (((sock = socket(PF_PACKET, SOCK_RAW, htons(0x88A4))) < 0) && tries < MAX_TRIES_SOCKET) {
		sleep(1);
		tries++;
	}
	
	if(sock < 0) {
		perror("Failed to create socket");
		return -1;
	}
	
	printf("Socket created: socket id: %d\n", sock);

	int index_ioctl;
	strncpy(ifr.ifr_name, interface, IFNAMSIZ);
	if ((index_ioctl = ioctl(sock, SIOCGIFINDEX, &ifr)) < 0) {
		perror("Cannot get interface index");
		close(sock);
		return -1;
	}
	printf("Got interface: index: %d\n", index_ioctl);
	
	int64_t timeout = TIMEOUT_NSEC;
	if(ioctl(sock,RTNET_RTIOC_TIMEOUT,&timeout) < 0) {
		perror("Cannot set timout");
		printf("Continue without timeout\n");
	}

	addr.sll_family   = AF_PACKET;
	addr.sll_protocol = htons(0x88A4);
	addr.sll_ifindex  = ifr.ifr_ifindex;

	if ((bind(sock, (struct sockaddr *)&addr, sizeof(addr))) < 0) {
		perror("Cannot bind to local ip/port");
		close(sock);
		return -1;
	}
	
	return sock;
}

int close_socket(struct netif *ni) {
	int ret = close(ni->socket_private);
	int tries = 1;
	while(ret < 0 && tries < MAX_TRIES_SOCKET) {
		ret = close(ni->socket_private);
		tries++;
		sleep(1);
	}
	if(ret < 0)
		perror("Failed to close socket");
	return ret;
}

struct eth_msg {
	u_int8_t  ether_dhost[ETH_ALEN];      /* destination eth addr */
	u_int8_t  ether_shost[ETH_ALEN];      /* source ether addr    */
	u_int8_t  ether_type [2];                 /* packet type ID field */
	unsigned char  data [MAX_ETH_DATA];
};

static bool low_level_output(struct EtherCAT_Frame * frame, struct netif * netif)
{
	bool result = false;
	struct eth_msg msg_to_send;
	int tel;
	for(tel = 0; tel<MAX_ETH_DATA; tel++)
		msg_to_send.data[tel] = 0x00;
	int len_dump = framedump(frame, msg_to_send.data, MAX_ETH_DATA);
	int msg_len = len_dump + ETH_ALEN + ETH_ALEN + 2;   
	if(len_dump) { 
		int sock = netif->socket_private;
		// Destination address is broadcast MAC address
		// FIXME Is this also valid for EtherCAT UDP ethernet frames?
		memset(msg_to_send.ether_dhost, 0xFF, ETH_ALEN);
		// Source address
		for(tel = 0; tel<ETH_ALEN; tel++)
			msg_to_send.ether_shost[tel] = (netif->hwaddr)[tel];
		// Type is ethercat
		msg_to_send.ether_type[0] = 0x88;
		msg_to_send.ether_type[1] = 0xA4;
	

		// The actual send
		int len_send = send(sock,(unsigned char *)&msg_to_send,msg_len  ,0);
		if(len_send < 0)
			ec_log(EC_LOG_FATAL, "low_level_output(): Cannot Send\n");
		else
			result = true;	      
	}
	else { // higher level protocol error. Attempt to map to much data in one ethernet frame
		ec_log(EC_LOG_FATAL, "EtherCAT fatal: message buffer overflow\n");
		// Release the message buffer again
	}

	return result;
}


static bool low_level_input(struct EtherCAT_Frame * frame, struct netif * netif) {

	unsigned char buffer_receive[MAX_ETH_DATA + HEADER_SIZE];
	struct eth_msg *msg_received = (struct eth_msg *)buffer_receive;
	//Receive message from socket
	int sock = netif->socket_private;

  int len_recv;
  int tries=0;
  static const int MAX_TRIES=3; // Maximum number of tries for recieving packets

  do {
      ++tries;
      len_recv = recv(sock,buffer_receive,sizeof(buffer_receive),0);
      if(len_recv < 0) {
          //perror("low_level_input: Cannot receive msg: ");
          ec_log(EC_LOG_ERROR, "low_level_input: Cannot receive msg: %d\n",len_recv);
          return false;
      }
             
      if (len_recv <= sizeof(ETH_ALEN + ETH_ALEN + 2)) {
          ec_log(EC_LOG_ERROR, "low_level_input: recieved runt packet: %d\n",len_recv);
          continue;
      }
         
      if ( (msg_received->ether_shost[4] != netif->hwaddr[4]) ) {
          ec_log(EC_LOG_ERROR, "low_level_input: got incorrect sequence number: %d, expected %d\n",
                 msg_received->ether_shost[4], netif->hwaddr[4]);
          continue;
      }
      else {
          break;
      }       
  } while(tries < MAX_TRIES);

  if (tries >= MAX_TRIES) {
      ec_log(EC_LOG_ERROR, "low_level_input: recieved too many bad packets: %d\n",len_recv);
		return false;
	}

	if ( ((msg_received->ether_type[0]) != 0x88) || (msg_received->ether_type[1]) != 0xA4) {
		ec_log(EC_LOG_ERROR, "low_level_input: No EtherCAT msg!\n");
		return false;
	}

	// build Ethercat Frame
	int succes = framebuild(frame,msg_received->data);
	if (succes != 0){
		// FIXME decent error handling here
		ec_log(EC_LOG_ERROR, "low_level_input: framebuilding failed!\n");
		return false;
	}
  
	return true;
}

// For thread safety: txandrx() can be called from multiple threads...
static pthread_mutex_t txandrx_mut;

static bool ec_rtdm_txandrx(struct EtherCAT_Frame * frame, struct netif * netif) {
	int tries = 0;
	while (tries < MAX_TRIES_TX) {
		pthread_mutex_lock (&txandrx_mut);
    netif->hwaddr[4]++;
		if (low_level_output(frame,netif)){
			if (low_level_input(frame,netif)){
				pthread_mutex_unlock(&txandrx_mut);
				return true;
			}
			else{
				ec_log(EC_LOG_ERROR, "low_level_txandrx: receiving failed\n");
				pthread_mutex_unlock(&txandrx_mut);
			}
		}
		else{
			ec_log(EC_LOG_ERROR, "low_level_txandrx: sending failed\n");
			pthread_mutex_unlock(&txandrx_mut);
		}
		tries++;
	}
	ec_log(EC_LOG_FATAL, "low_level_txandrx: failed: MAX_TRIES_TX: Giving up\n");
	return false;
}

struct netif* init_ec(const char * interface) {
	int sock = init_socket(interface);
	if(sock < 0) {
		ec_log(EC_LOG_FATAL,"Socket initialisation failed\n");
		return 0;
	}
	struct netif* ni = (struct netif*)malloc(sizeof(struct netif));
	ni->txandrx = ec_rtdm_txandrx;
	ni->socket_private = sock;
	//Mac-address
	ni->hwaddr[0] = 0x00; ni->hwaddr[2] = 0x00; ni->hwaddr[4] = 0x00;
	ni->hwaddr[1] = 0x00; ni->hwaddr[3] = 0x00; ni->hwaddr[5] = 0x00;
	return ni;

}
