// $Id: ethercat_defs.h,v 1.6 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_defs_h__
#define __ethercat_defs_h__

// NOTE: Contrary to all logic ethercat data is transmitted in Little
// Endian!

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_endian.h>
#include <string.h>

typedef cyg_int8   EC_SINT;
typedef cyg_uint8  EC_USINT;
typedef cyg_int16  EC_INT;
typedef cyg_uint16 EC_UINT;
typedef cyg_int32  EC_DINT;
typedef cyg_uint32 EC_UDINT;

inline unsigned char * host2nw(unsigned char * a_buffer, EC_SINT a_value) {
  *a_buffer++ = static_cast<unsigned char>(a_value);
  return a_buffer;
}

inline unsigned char * host2nw(unsigned char * a_buffer, EC_USINT a_value) {
  *a_buffer++ = static_cast<unsigned char>(a_value);
  return a_buffer;
}

inline unsigned char * host2nw(unsigned char * a_buffer, EC_INT a_value) {
  cyg_uint16 tmp = CYG_CPU_TO_LE16(a_value);
  memcpy(a_buffer, &tmp, sizeof(tmp));
  return a_buffer + sizeof(tmp);
}

inline unsigned char * host2nw(unsigned char * a_buffer, EC_UINT a_value) {
  cyg_uint16 tmp = CYG_CPU_TO_LE16(a_value);
  memcpy(a_buffer, &tmp, sizeof(tmp));
  return a_buffer + sizeof(tmp);
}

inline unsigned char * host2nw(unsigned char * a_buffer, EC_DINT a_value) {
  cyg_uint32 tmp = CYG_CPU_TO_LE32(a_value);
  memcpy(a_buffer, &tmp, sizeof(tmp));
  return a_buffer + sizeof(tmp);
}

inline unsigned char * host2nw(unsigned char * a_buffer, EC_UDINT a_value) {
  cyg_uint32 tmp = CYG_CPU_TO_LE32(a_value);
  memcpy(a_buffer, &tmp, sizeof(tmp));
  return a_buffer + sizeof(tmp);
}

inline unsigned char * host2nw(unsigned char * a_buffer, const unsigned char * a_data, size_t a_datalen) {
  memcpy(a_buffer, a_data, a_datalen);
  return a_buffer + a_datalen;
}

inline const unsigned char * nw2host(const unsigned char * a_data, EC_SINT& a_value) {
  a_value = static_cast<EC_SINT>(*a_data);
  return a_data + sizeof(a_value);
}

inline const unsigned char * nw2host(const unsigned char * a_data, EC_USINT& a_value) {
  a_value=  static_cast<EC_USINT>(*a_data);
  return a_data + sizeof(a_value);
}

inline const unsigned char * nw2host(const unsigned char * a_data, EC_INT& a_value) {
  cyg_uint16 tmp;
  memcpy(&tmp, a_data, sizeof(tmp));
  tmp = CYG_LE16_TO_CPU(tmp);

  a_value = static_cast<EC_INT>(tmp);
  return a_data + sizeof(a_value);
}

inline const unsigned char * nw2host(const unsigned char * a_data, EC_UINT& a_value) {
  cyg_uint16 tmp;
  memcpy(&tmp, a_data, sizeof(tmp));
  tmp = CYG_LE16_TO_CPU(tmp);

  a_value = static_cast<EC_UINT>(tmp);
  return a_data + sizeof(a_value);
}

inline const unsigned char * nw2host(const unsigned char * a_data, EC_DINT& a_value) {
  cyg_uint32 tmp;
  memcpy(&tmp, a_data, sizeof(tmp));
  tmp = CYG_LE32_TO_CPU(tmp);

  a_value = static_cast<EC_DINT>(tmp);
  return a_data + sizeof(a_value);
}

inline const unsigned char * nw2host(const unsigned char * a_data, EC_UDINT& a_value) {
  cyg_uint32 tmp;
  memcpy(&tmp, a_data,sizeof(tmp));
  tmp = CYG_LE32_TO_CPU(tmp);

  a_value = static_cast<EC_UDINT>(tmp);
  return a_data + sizeof(a_value);
}

inline const unsigned char * nw2host(const unsigned char * a_data, size_t a_datalen) {
  return a_data + a_datalen;
}

#endif // __ethercat_defs_h__
