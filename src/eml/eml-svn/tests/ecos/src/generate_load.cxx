// $Id: generate_load.cxx,v 1.4 2006/02/23 09:32:57 kgad Exp $
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
#include <cyg/kernel/kapi.h>
#include <stdlib.h>
#include <stdio.h>

#define CYGNUM_ETHERCATMASTER_LOAD_STACK_SIZE 16384
#define CYGNUM_ETHERCATMASTER_LOAD_PRIORITY (CYGNUM_ETHERCATMASTER_DELIVERY_PRIORITY + 15)

// Load thread data
static cyg_handle_t h_load_thread;
static cyg_thread     load_thread;
static unsigned char  load_thread_stack[CYGNUM_ETHERCATMASTER_LOAD_STACK_SIZE];

static void 
ethercat_load(cyg_addrword_t)
{
  // Generate load here... by printf() statement and sleep for a
  // random time...
  unsigned int rnd = 0;
  cyg_tick_count_t delay = 0;
  char s[]="Random ";
  unsigned int i;
  
  while (true)
    {
      // Get random value here...
      // Rand goes from zero to RANDMAX...
      rnd = rand()/10000000;
      printf("Sleeping %d ticks\n",rnd);
      delay = (cyg_tick_count_t) rnd;
      // delay = 100;
      cyg_thread_delay(delay);
      for ( i = 0; i < rnd ; i++){
	  printf("%s",s);
      }
    }
}

void 
cyg_load_init(void)
{
  static int is_initialised = false;
  if(is_initialised) return;

  // KG Check what this does...
  // cyg_do_net_init(); // Force linking in the initializing constructor
  
  // Start helper threads etc here
  cyg_thread_create(CYGNUM_ETHERCATMASTER_LOAD_PRIORITY,
                    ethercat_load, 0,
                    "EtherCAT Master Load",
                    load_thread_stack, 
		    CYGNUM_ETHERCATMASTER_LOAD_STACK_SIZE,
                    &h_load_thread, &load_thread);
  cyg_thread_resume(h_load_thread);

  // And bring up devices
  // cyg_net_init_devs();

  // Done
  is_initialised = true;
}


