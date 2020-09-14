// $Id: process_data.cxx,v 1.7 2006/02/23 09:32:57 kgad Exp $
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

#include <pkgconf/net_ethercatmaster.h>
#include <ethercat/ethercat_defs.h>
#include <al/ethercat_master.h>
#include <cyg/kernel/kapi.h>
#include <stdlib.h>
#include <stdio.h>

#define CYGNUM_ETHERCATMASTER_PD_STACK_SIZE 16384
// Should be less important than the delivery thread (running the
// DSR), but more important than the routing thread (and especially
// more important than the load thread!
#define CYGNUM_ETHERCATMASTER_PD_PRIORITY (CYGNUM_ETHERCATMASTER_DELIVERY_PRIORITY + 1)

/* BIG FAT WARNING:  The following thread priority is not for PRODUCTION use.
   Use it only to verify that the jitter does increase _tremendously_
   when the priority of the router and load-generation thread are
   higher than the priority of the PD thread!!
*/
// #define CYGNUM_ETHERCATMASTER_PD_PRIORITY 25

// Process Data thread data
static cyg_handle_t h_pd_thread;
static cyg_thread     pd_thread;
static unsigned char  pd_thread_stack[CYGNUM_ETHERCATMASTER_PD_STACK_SIZE];

// Thread period in nanoseconds
static const unsigned long long pd_thread_period_ns = 100000000;
// Convert thread period from nanoseconds to kernel ticks
static const cyg_tick_count_t pd_thread_period = (pd_thread_period_ns * CYGNUM_HAL_RTC_DENOMINATOR) / CYGNUM_HAL_RTC_NUMERATOR;

static const unsigned char LOGICAL_MSG_LENGTH = 0x02;

static void 
ethercat_pd(cyg_addrword_t)
{
  unsigned char logical_msg[LOGICAL_MSG_LENGTH] = {0xff, 0xff};
  EtherCAT_Master * EM = EtherCAT_Master::instance();

  EC_UINT lights=0x0001;
  unsigned int counter = 0;
  while (true){
    cyg_thread_delay(pd_thread_period);
    if (counter < 0xf) {
      lights = lights << 1;
      counter++;}
    else {
      counter = 0;
      lights = 0x0001;
    }
    logical_msg[0] = (unsigned char) (lights & 0x00ff);
    logical_msg[1] = (unsigned char) ((lights & 0xff00) >> 8);
    EM->txandrx_PD(0x02,logical_msg);
  }
}

static void 
ethercat_pd_stresstest(cyg_addrword_t)
{
  unsigned char logical_msg[LOGICAL_MSG_LENGTH] = {0xff, 0xff};
  EtherCAT_Master * EM = EtherCAT_Master::instance();
  while (true){
    cyg_thread_delay(1);
    // logical_msg[0] = 0xff;
    // logical_msg[1] = 0xff;
    EM->txandrx_PD(0x02,logical_msg);
  }
}

void 
cyg_pd_init(void)
{
  static int is_initialised = false;
  if(is_initialised) return;

  // Start helper threads etc here
  cyg_thread_create(CYGNUM_ETHERCATMASTER_PD_PRIORITY,
                    ethercat_pd, 0,
                    "EtherCAT Master Pd",
                    pd_thread_stack, 
		    CYGNUM_ETHERCATMASTER_PD_STACK_SIZE,
                    &h_pd_thread, &pd_thread);
  cyg_thread_resume(h_pd_thread);

  // Done
  is_initialised = true;
}


