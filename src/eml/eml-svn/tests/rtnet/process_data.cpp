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

#include <ethercat/ethercat_defs.h>
#include <al/ethercat_master.h>
#include <posix/time.h>
#include <posix/pthread.h>
#include <stdlib.h>
#include <stdio.h>

// Process Data thread data
static pthread_t pd_thread;
static pthread_attr_t pd_thread_attr;

// Thread period in nanoseconds
static const long int pd_thread_period_ns = 1000*1000*50*1;
static int pd_stop_flag = 0;

static const unsigned char LOGICAL_MSG_LENGTH = 0x02;

static void * ethercat_pd(void*) {
  unsigned char logical_msg[LOGICAL_MSG_LENGTH] = {0xff, 0xff};
  EtherCAT_Master * EM = EtherCAT_Master::instance();

  EC_UINT lights=0x0001;
  unsigned int counter = 0;
  while (pd_stop_flag == 0){
		unsigned long overrun = 0;
    int ret = pthread_wait_np(&overrun);
    if ( ret < 0 || overrun != 0 ) {
			printf("ret: %d overrun: %d", ret, overrun );
			return (void *) ret;
			}
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
  return 0;
}

static void ethercat_pd_stresstest(void) {
  unsigned char logical_msg[LOGICAL_MSG_LENGTH] = {0xff, 0xff};
  EtherCAT_Master * EM = EtherCAT_Master::instance();
  while (true){
    // logical_msg[0] = 0xff;
    // logical_msg[1] = 0xff;
    EM->txandrx_PD(0x02,logical_msg);
  }
}

void pd_init(long int myperiod) {
  static int is_initialised = false;
  if(is_initialised) return;

  // Start helper threads etc here
 pthread_create(&pd_thread,&pd_thread_attr,ethercat_pd,0);
 struct timespec period;
 period.tv_sec = 1;
 period.tv_nsec = pd_thread_period_ns;
 struct timespec now;
 clock_gettime(CLOCK_REALTIME,&now);
 now.tv_sec += 1;
 int ret = pthread_make_periodic_np(pd_thread, &now, &period);
 if (ret != 0)
   printf("pthread_make_periodic_np:  %d\n", ret);
  // Done
  is_initialised = true;
}

void pd_cleanup()
{
	pd_stop_flag = 1;
	void * ret;
	pthread_join(pd_thread, &ret);
}


