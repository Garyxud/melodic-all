// $Id: netif.h,v 1.13 2006/02/20 15:57:33 kgad Exp $
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

 
// $Id: netif.h,v 1.13 2006/02/20 15:57:33 kgad Exp $
#ifndef __netif_h__
#define __netif_h__

// Forward declarations
struct EtherCAT_Frame;
struct netif;

#include <stdlib.h>

#define ETHERCAT_DEVICE_NAME_MAX_LENGTH 256
// Size of MAC adresses expressed as a number of bytes 
#define MAC_ADDRESS_SIZE 6 

// Function prototypes
externC void if_attach(struct netif * netif);
externC int framedump(const struct EtherCAT_Frame * frame, unsigned char * buffer, size_t bufferlength);
externC int framebuild(struct EtherCAT_Frame * frame, const unsigned char * buffer);

/// Generic ethercat interface towards lower level drivers. 
/** It should be readily re-implemented for different OSes such as
    RTAI, linux, ...     etc. (For the ease of porting the interface
    is in C).
*/
struct netif {
  /// Transmit and receive an EtherCAT frame
  /** Implemented for ecos in low_level_txandrx() in 
      packages/io/eth/current/src/ethercatmaster/eth_drv.c
      and mapped in eth_drv_init()
   */
  bool (* txandrx)(struct EtherCAT_Frame * frame, struct netif * netif);

  /// Request for servicing from low level device driver
  /** Implemented for ecos in ethercat_delivery_request() in
      packages/net/ethercatmaster/current/src/ecos/support.cxx
      and mapped in if_attach() 
  */
  void (* service_request_indication)(struct netif * netif);

  /// The MAC address
  unsigned char hwaddr[MAC_ADDRESS_SIZE];
  /// Name of the device
  char device_name[ETHERCAT_DEVICE_NAME_MAX_LENGTH];
  /// Field to be set by the device driver
  void * devicedriver_private;
  /// Field to be set by the stack
  void * stack_private;
};

#endif // __netif_h__
