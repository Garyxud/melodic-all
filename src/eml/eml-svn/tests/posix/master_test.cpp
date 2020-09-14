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


#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>

#include <al/ethercat_master.h>
#include <al/ethercat_AL.h>
#include <al/ethercat_process_data.h>
#include <ethercat/ethercat_xenomai_drv.h>
#include <ethercat/netif.h>
#include <dll/ethercat_dll.h>
#include <dll/ethercat_frame.h>
#include <dll/ethercat_slave_memory.h>
#include <al/ethercat_slave_conf.h>
#include <al/ethercat_slave_handler.h>

struct netif *ni;

///SlaveConfig EK1100
EtherCAT_FMMU_Config fmmu_config_EK1100(0);
EtherCAT_PD_Config pd_config_EK1100(0);
EtherCAT_SlaveConfig EC_EK1100(0x044c2c52,0x00010000,0x03e9,&fmmu_config_EK1100,&pd_config_EK1100);

///SlaveConfig EL4102
EtherCAT_FMMU_Config fmmu_config_EL4102(2);
EC_FMMU fmmu0_EL4102(0x00080000,0x0001,0x00,0x00,0x080D,0x00,true,false,true);
EC_FMMU fmmu1_EL4102(0x00010000,0x0004,0x00,0x07,0x1000,0x00,false,true,true);

EC_SyncMan syncman_mbx0_EL4102(0x1800,246,EC_QUEUED,EC_WRITTEN_FROM_MASTER,true,false,false,false,false,false,EC_FIRST_BUFFER,true);
EC_SyncMan syncman_mbx1_EL4102(0x18f6,246,EC_QUEUED,EC_READ_FROM_MASTER,true,false,false,false,false,false,EC_FIRST_BUFFER,true);
EtherCAT_MbxConfig mbx_conf_EL4102 = {syncman_mbx0_EL4102, syncman_mbx1_EL4102};

EtherCAT_PD_Config pd_config_EL4102(2);
EC_SyncMan
 syncman0_EL4102(0x1000,4,EC_BUFFERED,EC_WRITTEN_FROM_MASTER,true,false,false,false,false,false,EC_FIRST_BUFFER,true);
EC_SyncMan
 syncman1_EL4102(0x1100,0,EC_BUFFERED,EC_READ_FROM_MASTER,true,false,false,false,false,false,EC_FIRST_BUFFER,true);
EtherCAT_SlaveConfig EC_EL4102(0x10063052,0x00000000,0x03ea,&fmmu_config_EL4102,&pd_config_EL4102,&mbx_conf_EL4102);


static void init_slave_db() {
	EtherCAT_SlaveDb * slave_db = EtherCAT_SlaveDb::instance(2);
	fmmu_config_EL4102[0] = fmmu0_EL4102;
	fmmu_config_EL4102[1] = fmmu1_EL4102;
	pd_config_EL4102[0] = syncman0_EL4102;
	pd_config_EL4102[1] = syncman1_EL4102;
	slave_db->set_conf(&EC_EL4102,0);
	slave_db->set_conf(&EC_EK1100,1);
	
}

static void* run_master(void * arg) {
	
	ni = init_ec((char*)arg);
	init_slave_db();
	if(ni != 0) {
		printf("Attach netif \n\n");
		EtherCAT_DataLinkLayer::instance()->attach(ni);
		printf("Master initializing \n\n");
		EtherCAT_Master * EM = EtherCAT_Master::instance();
		printf("Getting slave handler\n");
		EtherCAT_SlaveHandler * sh_ek1100 = EM->get_slave_handler(0x03e9);
		//sleep(1);
		printf("Setting EK1100 to OP STATE\n");
		if(sh_ek1100->to_state(EC_OP_STATE))
			printf("EK1100 succesfully set in OP STATE\n");
		else
			printf("\nfailed to set EK1100 in OP STATE\n");
		printf("Getting slave handler\n");
		EtherCAT_SlaveHandler * sh_el4102 = EM->get_slave_handler(0x03ea);
		printf("Setting EL4102 to OP STATE\n");
		if(sh_el4102->to_state(EC_OP_STATE))
			printf("EL4102 succesfully set in OP STATE\n");
		else
			printf("\nfailed to set EL4102 in OP STATE!!\n");
		
		printf("AL initializing \n\n");
		EtherCAT_AL * AL = EtherCAT_AL::instance();
		
		///Set Channel 1 to 5V
		unsigned char msg[2] = {0xff, 0x3f};
		if(AL->isReady()) {
			printf("Test: Set Channel 1 to 5V: \n\n");
			int count = 0;
			while(count<100000) {
				EM->txandrx_PD(sizeof(msg),msg);
				count++;
			}
			printf("Test done.\n");
		}
		
		close_socket(ni);
		
	}
}

void catch_signal(int sig)
{
   if(ni != 0)
		close_socket(ni);
	exit(1);
}

int main(int argc, char** argv)
{

	signal(SIGTERM, catch_signal);
   signal(SIGINT, catch_signal);
   signal(SIGHUP, catch_signal);
	mlockall(MCL_CURRENT|MCL_FUTURE);

	if (argc < 2) {
        printf("usage: %s <interface>\n", argv[0]);
        return 0;
    }

	pthread_t xenothread;
	pthread_attr_t xenothread_attr;

	//Set attributes of thread
	pthread_attr_init (& xenothread_attr);

	//Start thread
	if(pthread_create(&xenothread, &xenothread_attr, run_master, argv[1]) != 0) {
		printf("Cannot create thread EtherCatThread.\n");
		exit(1);
	}
   pthread_join( xenothread, 0 );
	return 0;
}
