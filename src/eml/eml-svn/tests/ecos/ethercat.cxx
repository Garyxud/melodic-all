// $Id: ethercat.cxx,v 1.12 2006/02/23 09:32:27 kgad Exp $
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

#include <stdio.h>
#include <dll/ethercat_dll.h>
#include <dll/ethercat_frame.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

// Generate some load (for latency testing)
extern void cyg_load_init(void);
// Generate process data
extern void cyg_pd_init(void);

// Initialize slave database
void init_slave_db(void);

int main()
{
  // DLL
  // EtherCAT_DataLinkLayer * dll = EtherCAT_DataLinkLayer::instance();
  
  // Init slave db
  init_slave_db();
  
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
	cyg_pd_init();
      }
    }
  }

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

  // GENERATE SOME LOAD FOR LATENCY TEST...
  // cyg_load_init();

  printf("Hello EtherCAT world\n");
  return 0;
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

