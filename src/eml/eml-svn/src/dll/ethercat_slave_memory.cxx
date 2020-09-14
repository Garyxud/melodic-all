// $Id: ethercat_slave_memory.cxx,v 1.10 2006/02/20 15:57:33 kgad Exp $
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

 
#include "dll/ethercat_slave_memory.h"

EC_DLInformation::EC_DLInformation(EC_USINT type,
				   EC_USINT revision,
				   EC_UINT build,
				   EC_USINT no_of_supp_fmmu_channels,
				   EC_USINT no_of_supp_syncman_channels,
				   EC_USINT ram_size,
				   bool fmmu_bit_operation_not_supp)
  : EC_DataStruct(EC_DLInformationSize),
    Type(type),
    Revision(revision),
    Build(build),
    NoOfSuppFmmuChannels(no_of_supp_fmmu_channels),
    NoOfSuppSyncManChannels(no_of_supp_fmmu_channels),
    RamSize(ram_size),
    FmmuBitOperationNotSupp(fmmu_bit_operation_not_supp){};

EC_DLInformation::EC_DLInformation(const unsigned char * a_buffer)
  : EC_DataStruct(EC_DLInformationSize)
{
  a_buffer = nw2host(a_buffer,Type);
  a_buffer = nw2host(a_buffer,Revision);
  a_buffer = nw2host(a_buffer,Build);
  a_buffer = nw2host(a_buffer,NoOfSuppFmmuChannels);
  a_buffer = nw2host(a_buffer,NoOfSuppSyncManChannels);
  a_buffer = nw2host(a_buffer,RamSize);
  EC_USINT reserved = 0x00;
  a_buffer = nw2host(a_buffer,reserved);
  EC_UINT fmmu_bit_op_not_supp = 0x0000;
  a_buffer = nw2host(a_buffer,fmmu_bit_op_not_supp);
  FmmuBitOperationNotSupp = bool (fmmu_bit_op_not_supp & 0x0001);
}

unsigned char * 
EC_DLInformation::dump(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer,Type);
  a_buffer = host2nw(a_buffer,Revision);
  a_buffer = host2nw(a_buffer,Build);
  a_buffer = host2nw(a_buffer,NoOfSuppFmmuChannels);
  a_buffer = host2nw(a_buffer,NoOfSuppSyncManChannels);
  a_buffer = host2nw(a_buffer,RamSize);
  EC_USINT reserved = 0x00;
  a_buffer = host2nw(a_buffer,reserved);
  EC_UINT fmmu_bit_op_not_supp = (EC_UINT) FmmuBitOperationNotSupp;
  a_buffer = host2nw(a_buffer,fmmu_bit_op_not_supp);
  EC_UINT reservedword = 0x0000;
  a_buffer = host2nw(a_buffer,reservedword);
  return a_buffer;
}
// ==================================================

EC_ALControl::EC_ALControl(EC_State state, bool ack)
  : EC_DataStruct(EC_Slave_RD[AL_Control].size),
    State(state),
    Acknowledge(ack){};

EC_ALControl::EC_ALControl(const unsigned char * a_buffer)
  : EC_DataStruct(EC_Slave_RD[AL_Control].size)
{
  ec_log(EC_LOG_ERROR, "Not implemented yet\n");
}

unsigned char * 
EC_ALControl::dump(unsigned char * a_buffer) const
{
  EC_USINT firstbyte = State | ((EC_USINT) Acknowledge << 4);
  a_buffer = host2nw(a_buffer,firstbyte);
  EC_USINT applicationspecific = 0x00;
  a_buffer = host2nw(a_buffer,applicationspecific);
  return a_buffer;
}

// ==================================================

EC_ALStatus::EC_ALStatus(EC_State state, bool change)
  : EC_DataStruct(EC_Slave_RD[AL_Status].size),
    State(state),
    Change(change){};

EC_ALStatus::EC_ALStatus(const unsigned char * a_buffer)
  : EC_DataStruct(EC_Slave_RD[AL_Status].size)
{
  EC_USINT firstbyte;
  a_buffer = nw2host(a_buffer,firstbyte);
  // FIXME This is not foolproof as long as EC_State is an enum.
  State = (EC_State) (firstbyte & 0x0f); // 4 LSB bits represent state
  Change = (bool) (firstbyte & 0x10); // 5the byte represents change
}

unsigned char * 
EC_ALStatus::dump(unsigned char * a_buffer) const
{
  EC_USINT firstbyte = State | (((EC_USINT) Change) << 4);
  a_buffer = host2nw(a_buffer,firstbyte);
  EC_USINT applicationspecific = 0x00;
  a_buffer = host2nw(a_buffer,applicationspecific);
  return a_buffer;
}
// ==================================================

EC_SIIControlStatus::EC_SIIControlStatus(bool eeprom_write_access,
					 bool eeprom_address_algorithm ,
					 bool read_op ,
					 bool write_op ,
					 bool reload_op ,
					 bool write_error ,
					 bool busy)
  : EC_DataStruct(EC_Slave_RD[SII_ControlStatus].size),
    EepromWriteAccess(eeprom_write_access),
    EepromAddressAlgorithm(eeprom_address_algorithm),
    ReadOp(read_op),
    WriteOp(write_op),
    ReloadOp(reload_op),
    WriteError(write_error),
    Busy(busy),
    AcknowledgeError(false)
{}

EC_SIIControlStatus::EC_SIIControlStatus(const unsigned char * a_buffer)
  : EC_DataStruct(EC_Slave_RD[SII_ControlStatus].size)
{
  EC_UINT byte;
  a_buffer = nw2host(a_buffer,byte);
  EepromWriteAccess = (bool) (byte & 0x01);
  EepromAddressAlgorithm = (bool) ((byte & 0x80) >> 7);
  ReadOp = (bool) ((byte & 0x100) >> 8);
  WriteOp = (bool) ((byte & 0x200) >> 9);
  ReloadOp = (bool) ((byte & 0x400) >> 10);
  AcknowledgeError = (bool) ((byte & 0x2000) >> 13);
  WriteError = (bool) ((byte & 0x4000) >> 14);
  Busy = (bool) ((byte & 0x8000) >> 15);
}

unsigned char *
EC_SIIControlStatus::dump(unsigned char * a_buffer) const
{
  EC_UINT byte = 0x00;
  byte |= (EC_UINT) EepromWriteAccess;
  byte |= ((EC_UINT) EepromAddressAlgorithm ) << 7;
  byte |= ((EC_UINT) ReadOp ) << 8;
  byte |= ((EC_UINT) WriteOp ) << 9;
  byte |= ((EC_UINT) ReloadOp) << 10;
  byte |= ((EC_UINT) AcknowledgeError) << 13;
  byte |= ((EC_UINT) WriteError) << 14;
  byte |= ((EC_UINT) Busy) << 15;
  return host2nw(a_buffer,byte);
}

// ==================================================
EC_FMMU::EC_FMMU(EC_UDINT logical_start_address,
		 EC_UINT length,
		 EC_BitPos logical_start_bit,
		 EC_BitPos logical_end_bit,
		 EC_UINT physical_start_address,
		 EC_BitPos physical_start_bit,
		 bool read_enable,
		 bool write_enable,
		 bool channel_enable)
  : EC_DataStruct(EC_Slave_RD[FMMU_0].size),
    LogicalStartAddress(logical_start_address),
    Length(length),
    LogicalStartBit(logical_start_bit),
    LogicalEndBit(logical_end_bit),
    PhysicalStartAddress(physical_start_address),
    PhysicalStartBit(physical_start_bit),
    ReadEnable(read_enable),
    WriteEnable(write_enable),
    ChannelEnable(channel_enable){};

EC_FMMU::EC_FMMU(const unsigned char * a_buffer)
  : EC_DataStruct(EC_Slave_RD[FMMU_0].size)
{
  ec_log(EC_LOG_ERROR, "Not implemented yet");
}

unsigned char *
EC_FMMU::dump(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer,LogicalStartAddress);
  a_buffer = host2nw(a_buffer,Length);
  a_buffer = host2nw(a_buffer,(EC_USINT) LogicalStartBit);
  a_buffer = host2nw(a_buffer,(EC_USINT) LogicalEndBit);
  a_buffer = host2nw(a_buffer,PhysicalStartAddress);
  a_buffer = host2nw(a_buffer,(EC_USINT) PhysicalStartBit);

  EC_USINT rw = ReadEnable | (((EC_USINT) WriteEnable) << 1);
  a_buffer = host2nw(a_buffer,rw);
  a_buffer = host2nw(a_buffer,(EC_USINT) ChannelEnable);
  EC_USINT reserved = 0x00;
  a_buffer = host2nw(a_buffer,reserved);
  a_buffer = host2nw(a_buffer,reserved);
  a_buffer = host2nw(a_buffer,reserved);
  return a_buffer;
}

// ==================================================

EC_SyncMan::EC_SyncMan(EC_UINT physical_start_address,
		       EC_UINT length,
		       EC_BufferType buffer_type,
		       EC_Direction direction,
		       bool AL_event_enable,
		       bool watchdog_enable,
		       bool write_event,
		       bool read_event,
		       bool watchdog_trigger,
		       bool queued_state,
		       EC_BufferedState buffered_state,
		       bool channel_enable)
  : EC_DataStruct(EC_Slave_RD[Sync_Manager_0].size),
    PhysicalStartAddress(physical_start_address),
    Length(length),
    BufferType(buffer_type),
    Direction(direction),
    ALEventEnable(AL_event_enable),
    ECATEventEnable(false),
    WatchdogEnable(watchdog_enable),
    WriteEvent(write_event),
    ReadEvent(read_event),
    WatchdogTrigger(watchdog_trigger),
    QueuedState(queued_state),
    BufferedState(buffered_state),
    ChannelEnable(channel_enable){};

EC_SyncMan::EC_SyncMan(const unsigned char * a_buffer)
  : EC_DataStruct(EC_Slave_RD[Sync_Manager_0].size)
{
  ec_log(EC_LOG_ERROR, "Not implemented yet\n");
}

unsigned char *
EC_SyncMan::dump(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer,PhysicalStartAddress);
  a_buffer = host2nw(a_buffer,Length);
  // fixme explicit cast necessary here?
  EC_USINT thirdbyte = BufferType | (Direction << 2) | ((EC_USINT) ECATEventEnable << 4) | ((EC_USINT) ALEventEnable << 5) | ((EC_USINT) WatchdogEnable << 6);
  a_buffer = host2nw(a_buffer,thirdbyte);
  EC_USINT fourthbyte = (EC_USINT) WriteEvent | ((EC_USINT) ReadEvent << 1) | ((EC_USINT) WatchdogTrigger << 2) | ((EC_USINT) QueuedState << 3) | (BufferedState << 4);
  a_buffer = host2nw(a_buffer,fourthbyte);
  a_buffer = host2nw(a_buffer,(EC_UINT) ChannelEnable);
  return a_buffer;
}

// ==================================================
  
