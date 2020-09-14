// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Martí Morta Garriga  (mmorta@iri.upc.edu)
// All rights reserved.
//
// Copyright (C) 2013 Jochen Sprickerhof <jochen@sprickerhof.de>
//
// This file is part of IRI EPOS2 Driver
// IRI EPOS2 Driver is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <cstdio>
#include <sstream>
#include "Epos2.h"
//#define DEBUG

// ----------------------------------------------------------------------------
//   CLASS
// ----------------------------------------------------------------------------
//     CONSTRUCTOR
// ----------------------------------------------------------------------------

CEpos2::CEpos2(int8_t nodeId) : node_id(nodeId), verbose(false)
{ }

//     DESTRUCTOR
// ----------------------------------------------------------------------------

CEpos2::~CEpos2()
{
}

// ----------------------------------------------------------------------------
//   OPERATION
// ----------------------------------------------------------------------------
//     INIT
// ----------------------------------------------------------------------------

void CEpos2::init()
{
  this->openDevice();
  this->readStatusWord();
}

//     CLOSE
// ----------------------------------------------------------------------------

void CEpos2::close()
{
  this->disableVoltage();
}

//     P (print for debug) (stringstream)
// ----------------------------------------------------------------------------

void CEpos2::p(const std::stringstream& text)
{
	if(this->verbose) std::cout << "    [EPOS2] " << text.str() << std::endl;
}

//     P (char *)
// ----------------------------------------------------------------------------

void CEpos2::p(const char *text)
{
  if(this->verbose) std::cout << "    [EPOS2] " << text << std::endl;
}

//     GET VERBOSE
// ----------------------------------------------------------------------------

bool CEpos2::getVerbose()
{
  return(this->verbose);
}

//     SET VERBOSE
// ----------------------------------------------------------------------------

void CEpos2::setVerbose(bool verbose)
{
  this->verbose=verbose;
}


//----------------------------------------------------------------------------
//   COMMUNICATION
// ----------------------------------------------------------------------------
//     OPEN DEVICE
// ----------------------------------------------------------------------------

bool CEpos2::ftdi_initialized = false;
Ftdi::Context CEpos2::ftdi;

void CEpos2::openDevice()
{
    if(CEpos2::ftdi_initialized)
      return;
    if(CEpos2::ftdi.open(0x403, 0xa8b0) != 0)
        throw EPOS2OpenException("No FTDI devices connected");

    CEpos2::ftdi.set_baud_rate(1000000);
    CEpos2::ftdi.set_line_property(BITS_8, STOP_BIT_1, NONE);
    CEpos2::ftdi.set_usb_read_timeout(10000);
    CEpos2::ftdi.set_usb_write_timeout(10000);
    CEpos2::ftdi.set_latency(1);
    CEpos2::ftdi_initialized = true;
}

//     READ OBJECT
// ----------------------------------------------------------------------------

int32_t CEpos2::readObject(int16_t index, int8_t subindex)
{
  int32_t result = 0x00000000;
  int16_t req_frame[4];
  uint16_t ans_frame[4];

  req_frame[0] = 0x0210;     // header (LEN,OPCODE)
  req_frame[1] = index;      // data
  req_frame[2] = ((0x0000 | this->node_id) << 8) | subindex; // node_id subindex
  req_frame[3] = 0x0000;     // CRC

  //p("readObject: sendFrame");
  this->sendFrame(req_frame);

  //printf("RF: %.2X %.2X %.2X %.2X\n",req_frame[0],req_frame[1],req_frame[2],req_frame[3]);

  //p("readObject: receiveFrame");
  this->receiveFrame(ans_frame);

  //printf("AF: %.2X %.2X %.2X %.2X\n",ans_frame[0],ans_frame[1],ans_frame[2],ans_frame[3]);

  // if 0x8090, its 16 bit answer else is 32 bit
  if(ans_frame[3]==0x8090)
    result = ans_frame[2];
  else
    result = (ans_frame[3] << 16) | ans_frame[2];

  //printf("result: %d %d -> %d\n",ans_frame[3],ans_frame[2],result);

  return result;
}

//     WRITE OBJECT
// ----------------------------------------------------------------------------

int32_t CEpos2::writeObject(int16_t index, int8_t subindex, int32_t data)
{
  int32_t result = 0;
  int16_t req_frame[6]={0,0,0,0,0,0};
  uint16_t ans_frame[40]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  req_frame[0] = 0x0411;     // header (LEN,OPCODE)
  req_frame[1] = index;      // data
  req_frame[2] = ((0x0000 | this->node_id) << 8) | subindex;
  req_frame[3] = data & 0x0000FFFF;
  req_frame[4] = data >> 16;
  req_frame[5] = 0x0000;     // checksum

  this->sendFrame(req_frame);
  this->receiveFrame(ans_frame);

  // if 0x8090, its 16 bit answer else is 32 bit
  if(ans_frame[3]==0x8090)
    result = ans_frame[2];
  else
    result = (ans_frame[3] << 16) | ans_frame[2];

  return result;
}

//     SEND FRAME
// ----------------------------------------------------------------------------

void CEpos2::sendFrame(int16_t *frame)
{
  uint8_t trans_frame[80];                  // transmission frame
  int16_t length = ((frame[0] & 0xFF00) >> 8 ) + 2;   // frame length

  // Add checksum to the frame
  frame[length-1] = this->computeChecksum(frame,length);

  // add SYNC characters (DLE and STX)
  trans_frame[0] = 0x90;
  trans_frame[1] = 0x02;

  // Stuffing
  int8_t i=0, tf_i=2;
  while( i < length )
  {
      // LSB
      trans_frame[tf_i] = frame[i] & 0x00FF;
      if( trans_frame[tf_i] == 0x90 )
      {
        tf_i++;
        trans_frame[tf_i] = 0x90;
      }
      tf_i++;

      // MSB
      trans_frame[tf_i] = (frame[i] & 0xFF00) >> 8;
      if( trans_frame[tf_i] == 0x90 )
      {
        tf_i++;
        trans_frame[tf_i] = 0x90;
      }
      tf_i++;
      i++;
  }

    if(CEpos2::ftdi.write(trans_frame, tf_i) < 0)
        throw EPOS2IOException("Impossible to write Status Word.\nIs the controller powered ?");
}

//     RECEIVE FRAME
// ----------------------------------------------------------------------------

void CEpos2::receiveFrame(uint16_t* ans_frame)
{
  // length variables
  uint16_t read_desired         = 0;       // length of data that must read
  uint16_t read_real            = 0;       // length of data read actually
  uint16_t Len                  = 0;       // Len header part in epos2 usb frame
  uint16_t read_point           = 0;       // Position of the data read
  uint16_t state                = 0;       // state of the parsing state machine
  bool packet_complete     = false;

  // data holders
  uint8_t *read_buffer = NULL;  // frame buffer stuffed
  uint8_t *data        = NULL;  // frame buffer unstuffed
  uint8_t cheksum[2];

  // get data packet
  do{

    read_desired = CEpos2::ftdi.read_chunk_size();

    read_buffer = new uint8_t[read_desired];

    read_real    = CEpos2::ftdi.read(read_buffer, read_desired);

    if(read_real < 0)
    {
      delete[] read_buffer;
      if(data!=NULL)
        delete[] data;
      throw EPOS2IOException("Impossible to read Status Word.\nIs the controller powered ?");
    }

    for(uint16_t i=0;i<read_real;i++)
    {
      switch (state)
      {
        case 0:
        // no sync
          if(read_buffer[i] == 0x90)
            state = 1;
          else
            state = 0;
          break;
        case 1:
          // sync stx
          if(read_buffer[i] == 0x02)
            state = 2;
          else
            state = 0;
          break;
        case 2:
          // opcode
          state = 3;
         break;
        case 3:
          // len (16 bits)
          Len = read_buffer[i];
          if(data!=NULL)
            delete[] data;
          data = new uint8_t[Len*2];
          read_point = -1;
          state = 4;
          break;
        case 4:
          read_point++;
          data[read_point] = read_buffer[i];
          if(data[read_point]==0x90)
          {
            state = 5;
          }else{
            if(read_point+1 == Len*2)
              state = 6;
            else
              state = 4;
          }
          break;
        case 5:
          // destuffing
            state = 4;
            break;
        case 6:
          // checksum 1
          cheksum[1] = read_buffer[i];
          if(cheksum[1]==0x90){
            state = 8;
          }else{
            state = 7;
          }
          break;
        case 7:
          // checksum 0
          cheksum[0] = read_buffer[i];
          if(cheksum[0]==0x90){
            state = 9;
          }else{
            state = 0;
            packet_complete = true;
          }
          break;
        case 8:
          // destuff checksum 1
          state = 7;
          break;
        case 9:
          // destuff checksum 0
          state = 0;
          packet_complete = true;
          break;
      }
    }

    delete[] read_buffer;

  }while(!packet_complete);


  // parse data
  int tf_i = 0;
  for(int i = 0; i < Len; i++)
  {
    ans_frame[i] = 0x0000;
    // LSB to 0x__··
    ans_frame[i] = data[tf_i];
    tf_i++;
    // MSB to 0x··__
    ans_frame[i] = (data[tf_i]<<8) | ans_frame[i];
    tf_i++;
  }

  if(data!=NULL)
    delete[] data;

}

//     COMPUTE CHECKSUM
// ----------------------------------------------------------------------------

int16_t CEpos2::computeChecksum(int16_t *pDataArray, int16_t numberOfWords)
{
  uint16_t shifter, c;
  uint16_t carry;
  uint16_t CRC = 0;

  //Calculate pDataArray Word by Word
  while(numberOfWords--)
  {
    shifter = 0x8000;                 //Initialize BitX to Bit15
    c = *pDataArray++;                //Copy next DataWord to c
    do
    {
      carry = CRC & 0x8000;    //Check if Bit15 of CRC is set
      CRC <<= 1;               //CRC = CRC * 2
      if(c & shifter) CRC++;   //CRC = CRC + 1, if BitX is set in c
      if(carry) CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
      shifter >>= 1;           //Set BitX to next lower Bit, shifter = shifter/2
    } while(shifter);
  }

  return (int16_t)CRC;
}


//----------------------------------------------------------------------------
//   MANAGEMENT
// ----------------------------------------------------------------------------
//     Get State
// ----------------------------------------------------------------------------

long CEpos2::getState()
{


	long ans = this->readObject(0x6041, 0x00);

  std::stringstream s;
  s << "Estat: " << ans << " /  std::dec= " <<std::dec<< ans;
  p(s);

	// OBTENIR EL NUMERO D'ESTAT
	bool bits[16];
	bits[0]=  (ans & 0x0001);
	bits[1]=  (ans & 0x0002);
	bits[2]=  (ans & 0x0004);
	bits[3]=  (ans & 0x0008);

	bits[4]=  (ans & 0x0010);
	bits[5]=  (ans & 0x0020);
	bits[6]=  (ans & 0x0040);
	bits[7]=  (ans & 0x0080);

	bits[8]=  ans & 0x0100;
	bits[9]=  ans & 0x0200;
	bits[10]= ans & 0x0400;
	bits[11]= ans & 0x0800;

	bits[12]= ans & 0x1000;
	bits[13]= ans & 0x2000;
	bits[14]= ans & 0x4000;
	bits[15]= ans & 0x8000;



  #ifdef DEBUG
	std::cout
	<< bits[15]
	<< bits[14]
	<< bits[13]
	<< bits[12]
	<< bits[11]
	<< bits[10]
	<< bits[9]
	<< bits[8]
	<< bits[7]
	<< bits[6]
	<< bits[5]
	<< bits[4]
	<< bits[3]
	<< bits[2]
	<< bits[1]
	<< bits[0]
	<< std::endl;
  #endif

	if(bits[14]){
		if(bits[4]){
      p("State: Measure Init");
			return(MEASURE_INIT);
		}else{
      p("State: Refresh");
			return(REFRESH);
		}
	}else{
		if(!bits[8]){
      p("State: Start");
			return(START);
		}else{
			if(bits[6]){
        p("State: Switch on disabled");
				return(SWITCH_ON_DISABLED);
			}else{
				if(bits[5]){
					if(bits[4]){
            p("State: Operation Enable");
						return(OPERATION_ENABLE);
					}else{
						if(bits[1]){
              p("State: Switched On");
							return(SWITCH_ON);
						}else{
              p("State: Ready to Switch On");
							return(READY_TO_SWITCH_ON);
						}
					}
				}else{
					if(!bits[3]){
						if(bits[2]){
              p("State: Quick Stop Active");
						return(QUICK_STOP);
						}else{
              p("State: Not Ready to Switch On");
							return(NOT_READY_TO_SWITCH_ON);
						}
					}else{
						if(bits[4]){
              p("State: Fault Reaction Active (Enabled)");
							return(QUICK_STOP_ACTIVE_ENABLE);
						}else{
							if(bits[2]){
                p("State: Fault Reaction Active (Disabled)");
								return(QUICK_STOP_ACTIVE_DISABLE);
							}else{
                p("State: Fault");
								return(FAULT);
							}
						}
					}
				}
			}
		}
	}
	// Error
  std::cout << this->searchErrorDescription( this->readError() ) << std::endl;
	throw EPOS2UnknownStateException(this->searchErrorDescription(this->readError()));
}

//     SHUTDOWN (transition)
// ----------------------------------------------------------------------------

void CEpos2::shutdown()
{
  this->writeObject(0x6040, 0x00, 0x06);
}

//     SWITCH ON (transition)
// ----------------------------------------------------------------------------

void CEpos2::switchOn()
{
  this->writeObject(0x6040, 0x00, 0x07);
}

//     DISABLE VOLTAGE (transition)
// ----------------------------------------------------------------------------

void CEpos2::disableVoltage()
{
  this->writeObject(0x6040, 0x00, 0x00);
}

//     QUICK STOP (transition)
// ----------------------------------------------------------------------------

void CEpos2::quickStop()
{
  this->writeObject(0x6040, 0x00, 0x02);
}

//     DISABLE OPERATION (transition)
// ----------------------------------------------------------------------------

void CEpos2::disableOperation()
{
  this->writeObject(0x6040, 0x00, 0x07);
}

//     ENABLE OPERATION (transition)
// ----------------------------------------------------------------------------

void CEpos2::enableOperation()
{
  this->writeObject(0x6040, 0x00, 0x0F);
}

//     FAULT RESET (transition)
// ----------------------------------------------------------------------------

void CEpos2::faultReset()
{
  this->writeObject(0x6040, 0x00, 0x80);
}

//----------------------------------------------------------------------------
//   OPERATION MODES
// ----------------------------------------------------------------------------
//     GET OPERATION MODE
// ----------------------------------------------------------------------------

long CEpos2::getOperationMode()
{
  long ans = this->readObject(0x6061, 0x00);

	// Rectificacio
  /// \todo veure si això fa falta
  //ans = this->getNegativeLong(ans);

	return(ans);
}

//     GET OPERATION MODE DESCRIPTION
// ----------------------------------------------------------------------------

std::string CEpos2::getOpModeDescription(long opmode)
{

	std::stringstream s;
	std::string       name;

	switch(opmode){
		case VELOCITY:
			name="Velocity";
		break;
		case POSITION:
			name="Position";
		break;
    case PROFILE_POSITION:
			name="Profile Position";
			break;
    case PROFILE_VELOCITY:
			name="Profile Velocity";
			break;
    case INTERPOLATED_PROFILE_POSITION:
			name="Interpolated Profile Position";
			break;
    case HOMING:
			name="Homing";
			break;
	}

	return(name);

}

//     SET OPERATION MODE
// ----------------------------------------------------------------------------

void CEpos2::setOperationMode(long opmode)
{
    this->writeObject(0x6060, 0x00,opmode);
}

//     ENABLE CONTROLLER
// ----------------------------------------------------------------------------

void CEpos2::enableController()
{
	int estat=0,timeout=0;
  bool controller_connected = false;

  estat = this->getState();

  while( !controller_connected && timeout<10 )
  {
    switch(estat)
    {
      case 0:
        // FAULT
        this->faultReset();
        timeout++;
        break;
      case 1:
        // START
        break;
      case 2:
        // NOT_READY_TO_SWITCH_ON
        break;
      case 3:
        // SWITCH_ON_DISABLED
        timeout++;
        this->shutdown();
        break;
      case 4:
        // READY_TO_SWITCH_ON
        this->switchOn();
        break;
      case 5:
        // SWITCH_ON
        controller_connected = true;
        break;
      case 6:
        // REFRESH
        break;
      case 7:
        // MEASURE_INIT
        break;
      case 8:
        // OPERATION_ENABLE
        this->disableOperation();
        break;
      case 9:
        // QUICK_STOP
        this->disableVoltage();
        break;
      case 10:
        // QUICK_STOP_ACTIVE_DISABLE
        break;
      case 11:
        // QUICK_STOP_ACTIVE_ENABLE
        break;
    }
    estat = this->getState();
  }

}

//     ENABLE MOTOR
// ----------------------------------------------------------------------------

void CEpos2::enableMotor(long opmode)
{
	int estat;

  estat = this->getState();

	if( estat == SWITCH_ON )
	{
    this->enableOperation();
	}

	if( opmode != NO_OPERATION )
	{
    if( this->getOperationMode() != opmode )
		{
      this->setOperationMode(opmode);
		}
	}
}


//     IS TARGET REACHED ? (Shared between modes)
// ----------------------------------------------------------------------------

bool CEpos2::isTargetReached()
{

  long ans = this->readObject(0x6041, 0x00);
  std::stringstream s;

	//s << "Estat: std::hex=" <<std::hex<< ans << " /  std::dec=" <<std::dec << ans;
  //this->p(s);
	// OBTENIR EL NUMERO D'ESTAT

	// bit10 = 0 , not reached
	// bit10 = 1 , reached

	return((bool)(ans & 0x0400));
}

//----------------------------------------------------------------------------
//   MODE VELOCITY
// ----------------------------------------------------------------------------
//     GET TARGET VELOCITY
// ----------------------------------------------------------------------------

long CEpos2::getTargetVelocity()
{
  return this->readObject(0x206B, 0x00);
}

//     SET TARGET VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::setTargetVelocity(long velocity)
{
  this->writeObject(0x206B, 0x00,velocity);
}

//     START VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::startVelocity()
{
  this->writeObject(0x6040, 0x00, 0x010f);
}

//     STOP VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::stopVelocity()
{
  // just velocity command = 0
  this->writeObject(0x206B, 0x00,0x0000);
}

//----------------------------------------------------------------------------
//   MODE PROFILE VELOCITY
// ----------------------------------------------------------------------------
//     GET TARGET PROFILE VELOCITY
// ----------------------------------------------------------------------------

long CEpos2::getTargetProfileVelocity()
{

  return this->readObject(0x60FF, 0x00);
}

//     SET TARGET PROFILE VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::setTargetProfileVelocity(long velocity)
{
  this->writeObject(0x60FF, 0x00, velocity);
}

//     START PROFILE VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::startProfileVelocity()
{
	int intmode = 0x000F;

  this->writeObject(0x6040, 0x00,intmode);
}

//     STOP PROFILE VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::stopProfileVelocity()
{
	int intmode = 0x010F;
  this->writeObject(0x6040, 0x00,intmode);
}

//----------------------------------------------------------------------------
//   MODE PROFILE POSITION
// ----------------------------------------------------------------------------
//     GET TARGET PROFILE POSITION
// ----------------------------------------------------------------------------

long CEpos2::getTargetProfilePosition()
{
  return this->readObject(0x607A, 0x00);
}

//     SET TARGET PROFILE POSITION
// ----------------------------------------------------------------------------

void CEpos2::setTargetProfilePosition(long position)
{
  this->writeObject(0x607A, 0x00,position);
}

// 0 halt, 1 abs, 2 rel
void CEpos2::startProfilePosition(epos_posmodes mode, bool blocking, bool wait, bool new_point)
 {

  int halt        = mode==HALT ?      0x0100 : 0x0000;
  int rel         = mode==RELATIVE ?  0x0040 : 0x0000;
  int nowait      = !wait ?           0x0020 : 0x0000;
  int newsetpoint = new_point ?       0x0010 : 0x0000;

  int intmode = 0x000F | halt | rel | nowait | newsetpoint;

  this->writeObject(0x6040, 0x00,intmode);

  if( blocking ){

    while( !this->isTargetReached() )
    {
      if(this->verbose) this->getMovementInfo();
      else usleep(1000);
    }
  }

}



// Current

long CEpos2::getTargetCurrent(){return 1;}

void CEpos2::setTargetCurrent(long current){}

void CEpos2::startCurrent(){}

void CEpos2::stopCurrent(){}



//----------------------------------------------------------------------------
//   CONTROL PARAMETERS
// ----------------------------------------------------------------------------

long CEpos2::getCurrentPGain()
{
  return this->readObject(0x60F6, 0x01);
}

void CEpos2::setCurrentPGain(long gain)
{
  this->writeObject(0x60F6, 0x01,gain);
}

long CEpos2::getCurrentIGain()
{
  return this->readObject(0x60F6, 0x02);
}

void CEpos2::setCurrentIGain(long gain)
{
  this->writeObject(0x60F6, 0x02,gain);
}

// Velocity

long CEpos2::getVelocityPGain()
{
  return this->readObject(0x60F9, 0x01);
}

void CEpos2::setVelocityPGain(long gain)
{
  this->writeObject(0x60F9, 0x01,gain);
}

long CEpos2::getVelocityIGain()
{
  return this->readObject(0x60F9, 0x02);
}

void CEpos2::setVelocityIGain(long gain)
{
  this->writeObject(0x60F9, 0x03,gain);
}

long CEpos2::getVelocitySetPointFactorPGain()
{
  return this->readObject(0x60F9, 0x03);
}

void CEpos2::setVelocitySetPointFactorPGain(long gain)
{
  this->writeObject(0x60F9, 0x03,gain);
}

// Position

long CEpos2::getPositionPGain()
{
  return this->readObject(0x60FB, 0x01);
}

void CEpos2::setPositionPGain(long gain)
{
  this->writeObject(0x60FB, 0x01,gain);
}

long CEpos2::getPositionIGain()
{
  return this->readObject(0x60FB, 0x02);
}

void CEpos2::setPositionIGain(long gain)
{
  this->writeObject(0x60FB, 0x02,gain);
}

long CEpos2::getPositionDGain()
{

  return this->readObject(0x60FB, 0x03);
}

void CEpos2::setPositionDGain(long gain)
{
  this->writeObject(0x60FB, 0x03,gain);
}

long CEpos2::getPositionVFFGain()
{
  return this->readObject(0x60FB, 0x04);
}

void CEpos2::setPositionVFFGain(long gain)
{
  this->writeObject(0x60FB, 0x04,gain);
}

long CEpos2::getPositionAFFGain()
{
  return this->readObject(0x60FB, 0x05);
}

void CEpos2::setPositionAFFGain(long gain)
{
  this->writeObject(0x60FB, 0x05,gain);
}

void CEpos2::getControlParameters(long &cp,long &ci,long &vp,long &vi,
                                  long &vspf, long &pp,long &pi,long &pd,
                                  long &pv,long &pa)
{
  cp = this->getCurrentPGain();
  ci = this->getCurrentIGain();
  vp = this->getVelocityPGain();
  vi = this->getVelocityIGain();
  vspf = this->getVelocitySetPointFactorPGain();
  pp = this->getPositionPGain();
  pi = this->getPositionIGain();
  pd = this->getPositionDGain();
  pv = this->getPositionVFFGain();
  pa = this->getPositionAFFGain();

  if(this->verbose) this->printControlParameters(cp,ci,vp,vi,vspf,pp,pi,pd,pv,pa);

}

void CEpos2::setControlParameters(long cp,long ci,long vp,long vi,long vspf,
                                  long pp,long pi,long pd,long pv,long pa)
{
  this->setCurrentPGain(cp);
  this->setCurrentIGain(ci);
  this->setVelocityPGain(vp);
  this->setVelocityIGain(vi);
  this->setVelocitySetPointFactorPGain(vspf);
  this->setPositionPGain(pp);
  this->setPositionIGain(pi);
  this->setPositionDGain(pd);
  this->setPositionVFFGain(pv);
  this->setPositionAFFGain(pa);


  this->getControlParameters(cp,ci,vp,vi,vspf,pp,pi,pd,pv,pa);

}

void CEpos2::printControlParameters(long cp,long ci,long vp,long vi,long vspf,
                                    long pp,long pi,long pd,long pv,long pa)
{
  std::cout << "    [EPOS2] Control Parameters:" << std::endl;
  std::cout << "    [EPOS2] Current:  P = " << cp << "  I = " << ci << std::endl;
  std::cout << "    [EPOS2] Velocity: P = " << vp << "  I = " << vi << "	SetPointFactorP = " << vspf << std::endl;
  std::cout << "    [EPOS2] Position: P = " << pp << "  I = " << pi << "	D = "<< pd << std::endl;
  std::cout << "    [EPOS2]           Vff = " << pv << "  Aff = " << pa << std::endl;
}

//----------------------------------------------------------------------------
//   PROFILE PARAMETERS
// ----------------------------------------------------------------------------

long CEpos2::getProfileVelocity(void)
{
  return this->readObject(0x6081, 0x00);
}

void CEpos2::setProfileVelocity(long velocity)
{
  this->writeObject(0x6081, 0x00,velocity);
}

long CEpos2::getProfileMaxVelocity(void)
{
  return this->readObject(0x607F, 0x00);
}

void CEpos2::setProfileMaxVelocity(long velocity)
{
  this->writeObject(0x607F, 0x00,velocity);
}

long CEpos2::getProfileAcceleration(void)
{
  return this->readObject(0x6083, 0x00);
}

void CEpos2::setProfileAcceleration(long acceleration)
{
  this->writeObject(0x6083, 0x00,acceleration);
}

long CEpos2::getProfileDeceleration(void)
{
  return this->readObject(0x6084, 0x00);
}

void CEpos2::setProfileDeceleration(long deceleration)
{
  this->writeObject(0x6084, 0x00,deceleration);
}

long CEpos2::getProfileQuickStopDecel(void)
{
  return this->readObject(0x6085, 0x00);
}

void CEpos2::setProfileQuickStopDecel(long deceleration)
{
  this->writeObject(0x6085, 0x00,deceleration);
}

long CEpos2::getProfileType(void)
{
  return this->readObject(0x6086, 0x00);
}

void CEpos2::setProfileType(long type)
{
  this->writeObject(0x6086, 0x00,type);
}

long CEpos2::getMaxAcceleration(void)
{
  return this->readObject(0x60C5, 0x00);
}

void CEpos2::setMaxAcceleration(long max_acceleration)
{
  this->writeObject(0x60C5, 0x00,max_acceleration);
}

void CEpos2::getProfileData(long &vel,long &maxvel,long &acc,long &dec,
                            long &qsdec, long &maxacc, long &type)
{
  vel    = this->getProfileVelocity();
  maxvel = this->getProfileMaxVelocity();
  acc    = this->getProfileAcceleration();
  dec    = this->getProfileDeceleration();
  qsdec  = this->getProfileQuickStopDecel();
  maxacc = this->getMaxAcceleration();
  type   = this->getProfileType();
}

void CEpos2::setProfileData(long vel,long maxvel,long acc,long dec,
                            long qsdec,long maxacc,long type)
{
  this->setProfileVelocity(vel);
  this->setProfileMaxVelocity(maxvel);
  this->setProfileAcceleration(acc);
  this->setProfileDeceleration(dec);
  this->setProfileQuickStopDecel(qsdec);
  this->setMaxAcceleration(maxacc);
  this->setProfileType(type);

  long v,m,a,d,q,ma,t;
  this->getProfileData(v,m,a,d,q,ma,t);
}

//----------------------------------------------------------------------------
//   READ INFORMATION
// ----------------------------------------------------------------------------

long CEpos2::readVelocity()
{
  return this->readObject(0x2028, 0x00);
}

long CEpos2::readVelocitySensorActual()
{
  return this->readObject(0x6069, 0x00);
}

long CEpos2::readVelocityDemand()
{
  return this->readObject(0x606B, 0x00);
}

long CEpos2::readVelocityActual	()
{
  return this->readObject(0x606C, 0x00);
}

long CEpos2::readCurrent()
{
  long ans = this->readObject(0x6078, 0x00);
  return this->getNegativeLong(ans);
}

long CEpos2::readCurrentAveraged()
{
  long ans = this->readObject(0x2027, 0x00);
  return this->getNegativeLong(ans);
}

long CEpos2::readCurrentDemanded()
{
  return this->readObject(0x2031, 0x00);
}

int32_t CEpos2::readPosition()
{
  return this->readObject(0x6064, 0x00);
}

long CEpos2::readStatusWord()
{
  return this->readObject(0x6041, 0x00);
}

long CEpos2::readEncoderCounter()
{
  return this->readObject(0x2020, 0x00);
}

long CEpos2::readEncoderCounterAtIndexPulse()
{
  return this->readObject(0x2021, 0x00);
}

long CEpos2::readHallsensorPattern()
{
  return this->readObject(0x2022, 0x00);
}

long CEpos2::readFollowingError()
{
  return this->readObject(0x20F4, 0x00);
}

void CEpos2::getMovementInfo()
{
	long vel_actual,vel_avg,vel_demand;
	int cur_actual,cur_avg,cur_demand;
	int32_t pos;
	bool verbose_status;

  verbose_status = this->verbose;
  this->setVerbose(false);
  vel_actual = this->readVelocityActual();
  vel_avg    = this->readVelocity();
  vel_demand = this->readVelocityDemand();

  cur_actual = this->readCurrent();
  cur_avg    = this->readCurrentAveraged();
  cur_demand = this->readCurrentDemanded();

  pos        = this->readPosition();
  this->setVerbose(verbose_status);

	printf("\r    [EPOS2] p: %ld v: %ld vavg: %ld vd: %ld c: %d cavg: %d cd: %d                   ",pos,vel_actual,vel_avg,vel_demand,cur_actual,cur_avg,cur_demand);
		fflush(stdout);

}

//----------------------------------------------------------------------------
//   ERRORS
// ----------------------------------------------------------------------------

char CEpos2::readError()
{
	char error_num=0;
  std::stringstream s;
  long ans = this->readObject(0x1001, 0x00);

	bool bits[8];
	bits[0]=  (ans & 0x0001);
	bits[1]=  (ans & 0x0002);
	bits[2]=  (ans & 0x0004);
	bits[3]=  (ans & 0x0008);

	bits[4]=  (ans & 0x0010);
	bits[5]=  (ans & 0x0020);
	bits[7]=  (ans & 0x0080);

	if(bits[7]) error_num=6; // Motion Error
	if(bits[5]) error_num=5; // Device profile specific
	if(bits[4]) error_num=4; // Communication Error

	if(bits[3]) error_num=3; // Temperature Error
	if(bits[2]) error_num=2; // Voltage Error
	if(bits[1]) error_num=1; // Current Error
	if(bits[0]) error_num=0; // Generic Error

  s << "Error: "<< error_num << " " << this->error_names[(unsigned char)error_num] <<
      " Value: 0x" <<std::hex<< ans << " , " <<std::dec<< ans;
  p(s);

	return(error_num);
}

void CEpos2::readErrorHistory(long *error[5])
{
	std::string error_des;

  long number_errors = this->readObject(0x1003, 0x00);
  std::cout << "  [EPOS2-ERROR] Number of Errors: " << number_errors << std::endl;

	// Read Errors
	for(int i=1;i<=number_errors;i++){
    long ans = this->readObject(0x1003, i);
		error[i] = &ans;
		error_des = this->searchErrorDescription(ans);

		std::cout << "  [EPOS2-ERROR] id: " << i << " : " << std::hex <<"0x"<< ans << " = " << error_des << std::endl;
	}
}

std::string CEpos2::searchErrorDescription(long error_code)
{
	int j=0;
	bool found = false;
  std::stringstream s;

  // error_codes length = 34

  while( !found && j < 34 ){
    if( error_code == this->error_codes[j] ){
			found = true;

      s << "Error Description "<< this->error_descriptions[j] << std::endl;
      p(s);
      return this->error_descriptions[j];

		}else{
			j++;
		}
	}
	if(!found) return "No Description for this error";
	else       return "Error Description";
}

//----------------------------------------------------------------------------
//   VERSIONS
// ----------------------------------------------------------------------------

long CEpos2::readVersionSoftware()
{
  return this->readObject(0x2003, 0x01);
}

long CEpos2::readVersionHardware()
{
  return this->readObject(0x2003, 0x02);
}



	// SENSOR CONFIGURATION

long CEpos2::getEncoderPulseNumber()
{
  return this->readObject(0x2210, 0x01);
}

void CEpos2::setEncoderPulseNumber(long pulses)
{
  this->writeObject(0x2210, 0x01, pulses);
}

long CEpos2::getEncoderType()
{return 1;}

void CEpos2::setEncoderType(long type)
{}

long CEpos2::getEncoderPolarity()
{return 1;}

void CEpos2::setEncoderPolarity(long polarity)
{}

void CEpos2::getEncoderParameters(long &pulses, long &type, long &polarity)
{}

void CEpos2::setEncoderParameters(long pulses, long type, long polarity)
{}

// Motor

long CEpos2::getMotorType(){return 1;}

void CEpos2::setMotorType(long type){}

long CEpos2::getMotorContinuousCurrentLimit(){return 1;}

void CEpos2::setMotorContinuousCurrentLimit(long current_mA){}

long CEpos2::getMotorOutputCurrentLimit(){return 1;}

void CEpos2::setMotorOutputCurrentLimit(long current_mA){}

long CEpos2::getMotorPolePairNumber(){return 1;}

void CEpos2::setMotorPolePairNumber(char pole_pairs){}

long CEpos2::getThermalTimeCtWinding(){return 1;}

void CEpos2::setThermalTimeCtWinding(long time_ds){}

//----------------------------------------------------------------------------
//   UTILITIES
// ----------------------------------------------------------------------------

long CEpos2::getMaxFollowingError()
{
  return this->readObject(0x6065, 0x00);
}

void CEpos2::setMaxFollowingError(long error)
{
  this->writeObject(0x6065, 0x00,error);
}

long CEpos2::getMinPositionLimit	()
{
  return this->readObject(0x607D, 0x01);
}

void CEpos2::setMinPositionLimit(long limit)
{
  this->writeObject(0x607D, 0x01,limit);
}


long CEpos2::getMaxPositionLimit	()
{
  return this->readObject(0x607D, 0x02);
}

void CEpos2::setMaxPositionLimit(long limit)
{
  this->writeObject(0x607D, 0x02,limit);
}

void CEpos2::disablePositionLimits(void)
{
	// min
	// -2147483647-1
  this->writeObject(0x607D, 0x01, -2147483647-1);
	// max
  this->writeObject(0x607D, 0x02, 2147483647);
}

long CEpos2::getPositionWindow(){return 1;}

void CEpos2::setPositionWindow(long window_qc){}

long CEpos2::getPositionWindowTime(){return 1;}

void CEpos2::setPositionWindowTime(long time_ms){}

long CEpos2::getVelocityWindow(){return 1;}

void CEpos2::setVelocityWindow(long window_rm){}

long CEpos2::getVelocityWindowTime(){return 1;}

void CEpos2::setVelocityWindowTime(long time_ms){}

long CEpos2::getPositionNotationIndex(){return 1;}

void CEpos2::setPositionNotationIndex(long notation){}

long CEpos2::getVelocityNotationIndex(){return 1;}

void CEpos2::setVelocityNotationIndex(long notation){}

long CEpos2::getAccelerationNotationIndex(){return 1;}

void CEpos2::setAccelerationNotationIndex(long notation){}

long CEpos2::getPositionDimensionIndex(){return 1;}

void CEpos2::setPositionDimensionIndex(long Dimension){}

long CEpos2::getVelocityDimensionIndex(){return 1;}

void CEpos2::setVelocityDimensionIndex(long Dimension){}

long CEpos2::getAccelerationDimensionIndex(){return 1;}

void CEpos2::setAccelerationDimensionIndex(long Dimension){}


void CEpos2::saveParameters()
{
  this->writeObject(0x1010, 0x01, 0x65766173);

}

void CEpos2::restoreDefaultParameters()
{
  this->writeObject(0x1011, 0x01, 0x64616F6C);

}

long CEpos2::getRS232Baudrate()
{
  return this->readObject(0x2002, 0x00);
}

void CEpos2::setRS232Baudrate(long baudrate)
{
  this->writeObject(0x2002, 0x00, baudrate);
}

long CEpos2::getRS232FrameTimeout()
{
  return this->readObject(0x2005, 0x00);
}

void CEpos2::setRS232FrameTimeout(long timeout)
{
  this->writeObject(0x2005, 0x00, timeout);
}

long CEpos2::getUSBFrameTimeout()
{
  return this->readObject(0x2006, 0x00);
}

void CEpos2::setUSBFrameTimeout(long timeout)
{
  this->writeObject(0x2006, 0x00, timeout);
}

long CEpos2::getNegativeLong(long positiu)
{
  if(positiu>32767){
    return(positiu-65536);
  }else{
    return(positiu);
  }
}


// #############################   I/O   ######################################

long CEpos2::getAnalogOutput1(){return 1;}

void CEpos2::setAnalogOutput1(long voltage_mV){}

// #############################   MARKER POSITION   ##########################

long CEpos2::getPositionMarker(int buffer)
{
  int obj;
  switch(buffer)
  {
    case 0:
      obj = 1;
      break;
    case 1:
      obj = 5;
      break;
    case 2:
      obj = 6;
      break;
  }
  return this->readObject(0x2074, obj);
}

void CEpos2::setPositionMarker(char mode, char polarity, char edge_type, char digitalIN)
{
  // set the digital input as position marker & options
  this->writeObject(0x2070, digitalIN, 4);
  // mask (which functionalities are active) (bit 3 0x0008)
  this->writeObject(0x2071, 0x02, 0x0008);
  // execution (if set the function executes) (bit 3 0x0008)
  this->writeObject(0x2071, 0x04, 0x0008);

  // options
  this->writeObject(0x2071, 0x03, polarity);
  this->writeObject(0x2074, 0x02, edge_type);
  this->writeObject(0x2074, 0x03, mode);

}

void CEpos2::waitPositionMarker()
{
  long markpos = this->getPositionMarker();

  while(markpos == this->getPositionMarker())
  {
    // minimum freq = 0.05s = 20Hz
    usleep(1000000*0.05);
  }
}


// #############################   HOMING   ###################################



void CEpos2::setHoming(int home_method, int speed_pos, int speed_zero,
                       int acc, int digitalIN)
{
  // set digital input as home switch
  this->writeObject(0x2070, digitalIN, 3);
  // mask
  this->writeObject(0x2071, 0x02, 0x0004);
  this->writeObject(0x2071, 0x04, 0x000C);
  // options
  this->writeObject(0x6098, 0x00, home_method);
  this->writeObject(0x6099, 0x01, speed_pos);
  this->writeObject(0x6099, 0x02, speed_zero);
  this->writeObject(0x609A, 0x00, acc);
}

void CEpos2::doHoming(bool blocking)
{
  this->writeObject(0x6040, 0x00, 0x001F);
  if(blocking)
  {
    while(!this->isTargetReached())
      usleep(1000000*0.05);
  }
}

void CEpos2::stopHoming()
{
  this->writeObject(0x6040, 0x00, 0x010F);
}

// #############################   DIG IN   ###################################





int CEpos2::getDigInState(int digitalIN)
{
  return this->readObject(0x2070, digitalIN);
}

int CEpos2::getDigInStateMask()
{
  return this->readObject(0x2071, 0x01);
}

int CEpos2::getDigInFuncMask()
{
  return this->readObject(0x2071, 0x02);
}

int CEpos2::getDigInPolarity()
{
  return this->readObject(0x2071, 0x03);
}

int CEpos2::getDigInExecutionMask()
{
  return this->readObject(0x2071, 0x04);
}


// deprecated


void CEpos2::setHomePosition(long home_position_qc)
{
  this->writeObject(0x2081, 0x00,home_position_qc);
}
long CEpos2::getHomePosition()
{
  return this->readObject(0x2081, 0x00);
}

void CEpos2::setHome()
{
  char c;
  int32_t home_pos=0;

  long mode_anterior = this->getOperationMode();
  this->disableOperation();
  std::cout << "    [EPOS2] Move Load to 0 position and press a key ";
  std::cin >> c;
  std::cout << std::endl;
  std::cout << "    [EPOS2] Wait until process finishes" << std::endl;
  this->enableOperation();
  home_pos = this->readPosition();
  this->setOperationMode(HOMING);
  this->getOperationMode();
  this->setHomePosition(home_pos);
  this->getHomePosition();
  this->setOperationMode(mode_anterior);
  this->getOperationMode();
  std::cout << "    [EPOS2] Restart EPOS2 (unplug from current) after that the new home will be set" << std::endl;
}

const std::string CEpos2::error_names[] = {
  "Generic Error",
  "Current Error",
  "Voltage Error",
  "Temperature Error",
  "Communication Error",
  "Device profile specific",
  "Motion Error"
};

const int CEpos2::error_codes[]={
  0x0000,
  0x1000,
  0x2310,
  0x3210,
  0x3220,
  0x4210,
  0x5113,
  0x5114,
  0x6100,
  0x6320,
  0x7320,
  0x8110,
  0x8111,
  0x8120,
  0x8130,
  0x8150,
  0x81FD,
  0x81FE,
  0x81FF,
  0x8210,
  0x8611,
  0xFF01,
  0xFF02,
  0xFF03,
  0xFF04,
  0xFF05,
  0xFF06,
  0xFF07,
  0xFF08,
  0xFF09,
  0xFF0A,
  0xFF0B,
  0xFF0C,
  0xFF0D
};

const std::string CEpos2::error_descriptions[]={
  "No Error",
  "Generic Error",
  "Over Current Error",
  "Over Voltage Error",
  "Under Voltage",
  "Over Temperature",
  "Supply Voltage (+5V) Too Low",
  "Supply Voltage Output Stage Too Low",
  "Internal Software Error",
  "Software Parameter Error",
  "Sensor Position Error",
  "CAN Overrun Error (Objects lost)",
  "CAN Overrun Error",
  "CAN Passive Mode Error",
  "CAN Life Guard Error",
  "CAN Transmit COB-ID collision",
  "CAN Bus Off",
  "CAN Rx Queue Overrun",
  "CAN Tx Queue Overrun",
  "CAN PDO length Error",
  "Following Error",
  "Hall Sensor Error",
  "Index Processing Error",
  "Encoder Resolution Error",
  "Hallsensor not found Error",
  "Negative Limit Error",
  "Positive Limit Error",
  "Hall Angle detection Error",
  "Software Position Limit Error",
  "Position Sensor Breach",
  "System Overloaded",
  "Interpolated Position Mode Error",
  "Autotuning Identification Error"
};
