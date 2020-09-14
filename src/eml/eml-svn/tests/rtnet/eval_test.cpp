// $Id: $
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

#include <posix/pthread.h>
#include <native/task.h>
#include <signal.h>
#include <sys/mman.h>

#include <ethercat/ethercat_xenomai_drv.h>
#include <ethercat/netif.h>
#include <stdio.h>
#include <stdlib.h>
#include <dll/ethercat_dll.h>
#include <unistd.h>
#include <dll/ethercat_frame.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

struct netif *ni;

// Generate process data
extern void pd_init(long);
extern void pd_cleanup();

// Initialize slave database
void init_slave_db(void);

void* eval_master(void*);
void catch_signal(int sig);

int main(int argc, char** argv) {

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
	signal(SIGHUP, catch_signal);
	mlockall(MCL_CURRENT|MCL_FUTURE);

	if (argc < 3) {
        printf("usage: %s <interface> <period (sec)>\n", argv[0]);
        return 0;
    }

	pthread_t eval_thread;
	pthread_attr_t eval_thread_attr;

	//Set attributes of thread
	pthread_attr_init (& eval_thread_attr);

	if(pthread_create(&eval_thread, &eval_thread_attr, eval_master, argv) != 0) {
		printf("Cannot create thread EtherCatThread.\n");
		exit(1);
	}
  sleep(1000);
  pthread_join( eval_thread, 0 );

  // wait for master cleanup
  pd_cleanup();
	// cleanup socket
  catch_signal(0);	
	return 0;
}

void* eval_master(void* arg) {
	ni = init_ec(((char**)arg)[1]);
  init_slave_db();

	printf("Attach netif \n\n");
	EtherCAT_DataLinkLayer::instance()->attach(ni);  
  printf("Creating Master\n");
  EtherCAT_Master * EM = EtherCAT_Master::instance();
  printf("Getting Slave Handler\n");
  EtherCAT_SlaveHandler * sh = EM->get_slave_handler(0x03e9);
  bool succeed = sh->to_state(EC_OP_STATE);
  if (succeed) {
    printf("EVA board now in OP state\n");
    succeed = sh->to_state(EC_INIT_STATE);
    if (succeed) {
      printf("And now back in INIT state\n");
      // UNCOMMENT THE FOLLOWING LINE IF YOU WANT TO VERIFY THAT NO
      // PROCESS DATA IS SENT IF THE BOARD IS NOT IN ITS OPERATIONAL
      // STATE!! 
      bool succeed = sh->to_state(EC_OP_STATE);
      if (succeed) {
	printf("And up and operational again\n");
	printf("Sending PD, look at the lights...\n");
	float period_f = atof(((char**)arg)[2]);
	int long period =(int long) (period_f*1000*1000*1000); 
	//printf("period: %d\n",period);
	pd_init(period);
      }
    }
  }

  printf("Hello EtherCAT world\n");
  return 0;

  // For testing mbx functionality as much as possible...
  /*
  EtherCAT_SlaveHandler * csh = EM->get_slave_handler(0x03ea);
  succeed = csh->to_state(EC_PREOP_STATE);
  if (succeed) {
    printf("Complex slave now in PREOP state\n");
    succeed = csh->to_state(EC_INIT_STATE);
    if (succeed) {
      printf("And now back in INIT state\n");
        bool succeed = csh->to_state(EC_SAFEOP_STATE);
	if (succeed) {
	  printf("And to SAFEOP\n");
	}
    }
  }
  */

}

// Configuration data of EVA board
EC_FMMU fmmu0(0x00010000,0x0002,0x00,0x07,0x1100,0x00,false,true,true);
EC_FMMU fmmu1(0x00010000,0x0002,0x00,0x07,0x1000,0x00,true,false,true);
EtherCAT_FMMU_Config fmmu_conf(2);
EC_SyncMan SyncMan0(0x1100,0x0002,
		    EC_BUFFERED,EC_WRITTEN_FROM_MASTER,false,true,
		    false,false,false,EC_QUEUED_STATE_READ,EC_FIRST_BUFFER,
		    true);
EC_SyncMan SyncMan1(0x1000,0x0002,
		    EC_BUFFERED,EC_READ_FROM_MASTER,false,false,
		    false,false,false,EC_QUEUED_STATE_READ,EC_FIRST_BUFFER,
		    true);
EtherCAT_PD_Config pd_conf(2);

EtherCAT_SlaveConfig EC_evaboard(0x26483052,0x000104b0,0x03e9,&fmmu_conf,&pd_conf);

// Trying to create mbx for testing some of the mbx software... 
/*
EC_SyncMan SyncMan_mbx0(0x1400,0x0100,
			   EC_QUEUED,EC_WRITTEN_FROM_MASTER,false,true,
			   false,false,false,EC_QUEUED_STATE_READ,EC_FIRST_BUFFER,
			   true);
EC_SyncMan SyncMan_mbx1(0x1500,0x0100,
			   EC_QUEUED,EC_READ_FROM_MASTER,false,false,
			   false,false,false,EC_QUEUED_STATE_READ,EC_FIRST_BUFFER,
			   true);
EtherCAT_MbxConfig mbx_conf = { SyncMan_mbx0, SyncMan_mbx0 };
EtherCAT_SlaveConfig EC_EL4102(0x10063052,0x270b0000,0x03ea,&fmmu_conf,&pd_conf,&mbx_conf);
*/

void init_slave_db(void)
{
  EtherCAT_SlaveDb * slave_db = EtherCAT_SlaveDb::instance(1);
  fmmu_conf[0] = fmmu0; fmmu_conf[1] = fmmu1;
  pd_conf[0] = SyncMan0; pd_conf[1] = SyncMan1;
  slave_db->set_conf(&EC_evaboard,0);
  
  // For testing mbx...
  /*
  EtherCAT_SlaveDb * slave_db = EtherCAT_SlaveDb::instance(2);
  fmmu_conf[0] = fmmu0; fmmu_conf[1] = fmmu1;
  pd_conf[0] = SyncMan0; pd_conf[1] = SyncMan1;
  slave_db->set_conf(&EC_evaboard,0);
  slave_db->set_conf(&EC_EL4102,1);
  */
}

void catch_signal(int sig) {
    if(ni != 0) {
		while ((close_socket(ni) < 0) && (errno == EAGAIN)) {
        	printf("socket busy - waiting...\n");
        	sleep(1);
    	}
	}
	exit(1);
}
