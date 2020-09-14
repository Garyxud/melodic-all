#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "helpers.h"
#include "udpmon.h"

#include <signal.h>

#include "ros/ros.h"
#include "network_monitor_udp/UdpSink.h"

struct udpmon_pkt buff;
unsigned char magic_udp[] = UDPMON_MAGIC;
unsigned char magic_ros[] = UDPMON_MAGIC_ROS;

#define FEEDBACK_QUEUE_SIZE 10000

int main(int argc, char **argv)
{
  ros::init(argc, argv, "udpmonsink");
  if (argc != 2) { fprintf(stderr, "usage: udpmonsink <port_to_listen_on>\n"); return 1; }

  ros::NodeHandle n;
  ros::Publisher sink_pub = n.advertise<network_monitor_udp::UdpSink>("udpsink_feedback", FEEDBACK_QUEUE_SIZE);

  int udp_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in port;
  int len;

  if (udp_socket == -1) { perror("socket"); return 1; }

  port.sin_family = AF_INET;
  port.sin_port = htons(atoi(argv[1]));
  port.sin_addr.s_addr = INADDR_ANY;

  if (bind(udp_socket, (struct sockaddr *) &port, sizeof(port))) { perror("bind"); return 1; }

  // Allow SIGINT to interrupt recvfrom
  struct sigaction sig;
  sigaction(SIGINT, NULL, &sig);
  sig.sa_flags &= ~SA_RESTART;
  sigaction(SIGINT, &sig, NULL);

  while ( ros::ok() ) 
  { 
    struct sockaddr src_addr;
    socklen_t addrlen = sizeof(src_addr);

    network_monitor_udp::UdpSink sink_msg;

    if ((len = recvfrom(udp_socket, &buff,  sizeof(buff), MSG_WAITALL, &src_addr, &addrlen)) == -1) 
    { 
      ros::shutdown();
      perror("recv"); 
      return 1;
    }

    if ( (*((int *) &magic_udp) == (*((int *) &buff.magic))) || 
         (*((int *) &magic_ros) == (*((int *) &buff.magic))) )
      buff.echoed = gettime();
    else 
      continue;

    if (*((int *) &magic_udp) == (*((int *) &buff.magic))) 
      sendto(udp_socket, &buff, len, 0, &src_addr, addrlen);
    else 
      {        
        for (int i=0; i<4; i++)
          sink_msg.magic[i] = (uint8_t)buff.magic[i];
        sink_msg.send_time = buff.sent;
        sink_msg.echo_time = buff.echoed;
        sink_msg.seqnum = buff.seqnum;
        sink_msg.source_id = buff.source_id;
        
        sink_pub.publish(sink_msg);
      }
  }
  
  return 0;
}
