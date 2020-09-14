#include "seed_smartactuator_sdk/seed3_command.h"
#include <iostream> // for cout/cerr
using namespace seed;
using namespace controller;

//#define DEBUG

///////////////////////////////
SerialCommunication::SerialCommunication()
:  io_(),serial_(io_),timer_(io_),is_canceled_(false)
{
}

///////////////////////////////
SerialCommunication::~SerialCommunication()
{
  if(serial_.is_open())serial_.close();
}

///////////////////////////////
bool SerialCommunication::openPort(std::string _port, unsigned int _baud_rate)
{
  boost::system::error_code error_code;
  serial_.open(_port,error_code);
  if(error_code){
    return false;
  }
  else{
    serial_.set_option(serial_port_base::baud_rate(_baud_rate));
    return true;
  }
}

///////////////////////////////
void SerialCommunication::closePort()
{
  if(serial_.is_open())serial_.close();
}

///////////////////////////////
void SerialCommunication::writeBuffer(std::vector<char>& _send_data)
{
  serial_mtx_.lock();
  boost::asio::write(serial_,buffer(_send_data));
  serial_mtx_.unlock();
}

///////////////////////////////
std::string SerialCommunication::readBuffer(uint16_t _wait_time)
{
  serial_mtx_.lock();
  usleep(_wait_time*1000);

  boost::asio::streambuf buffer;
  boost::asio::read_until(serial_,buffer, "\r");
  const std::string result = boost::asio::buffer_cast<const char*>(buffer.data());

  buffer.consume(buffer.size());

  serial_mtx_.unlock();
  return result;
}

///////////////////////////////
void SerialCommunication::onReceive(const boost::system::error_code& _error, size_t _bytes_transferred)
{
  if (_error && _error != boost::asio::error::eof) {
#if DEBUG
      std::cout << "receive failed: " << std::endl;
#endif
  }
  else {
      const std::string data(boost::asio::buffer_cast<const char*>(stream_buffer_.data()), stream_buffer_.size());
      receive_buffer_ = data;

      stream_buffer_.consume(stream_buffer_.size());
      timer_.cancel();
      is_canceled_ = true;
  }
}

///////////////////////////////
void SerialCommunication::onTimer(const boost::system::error_code& _error)
{
    if (!_error && !is_canceled_) serial_.cancel();
}

///////////////////////////////
void SerialCommunication::readBufferAsync(std::string _delim="\r", uint16_t _timeout)
{
  receive_buffer_.clear();
  is_canceled_ = false;

  boost::asio::async_read_until(serial_,stream_buffer_,_delim,
      boost::bind(&SerialCommunication::onReceive, this,
          boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  timer_.expires_from_now(boost::posix_time::milliseconds(_timeout));
  timer_.async_wait(boost::bind(&SerialCommunication::onTimer, this, _1));
  io_.reset();
  io_.run();
}

///////////////////////////////
void SerialCommunication::flushPort()
{
  ::tcflush(serial_.lowest_layer().native_handle(),TCIOFLUSH);
}
//*******************************************************************
//*******************************************************************
///////////////////////////////
SeedCommand::SeedCommand()
: check_sum_(0),count_(0),length_(0),serial_com_()
{
  send_data_.resize(6);
}

///////////////////////////////
SeedCommand::~SeedCommand()
{
  closeCom();
}

//////////////////////////////
bool SeedCommand::openPort(std::string _port, unsigned int _baud_rate){
    if(serial_com_.openPort(_port, _baud_rate)) is_open_ = true;
    else is_open_ = false;

    return is_open_;
}

//////////////////////////////
void SeedCommand::closePort(){
    serial_com_.closePort();
    is_open_ = false;
}

///////////////////////////////
void SeedCommand::openCom()
{
  std::vector<char> send_char;

  length_ = 5;
  send_char.resize(length_);
  fill(send_char.begin(),send_char.end(),0);

  send_char[0] = 'Z';
  send_char[1] = '0';
  send_char[2] = '\r';
  send_char[3] = 'O';
  send_char[4] = '\r';

  serial_com_.flushPort();
  serial_com_.writeBuffer(send_char);

  std::vector<uint8_t> receive_data;
  readSerialCommand(receive_data);
}

///////////////////////////////
void SeedCommand::closeCom()
{
  std::vector<char> send_char;

  length_ = 2;
  send_char.resize(length_);
  fill(send_char.begin(),send_char.end(),0);

  send_char[0] = 'C';
  send_char[1] = '\r';

  serial_com_.flushPort();
  serial_com_.writeBuffer(send_char);
}

///////////////////////////////
void SeedCommand::writeSerialCommand(uint8_t _id, uint8_t *_data)
{
  std::vector<char> send_char;
  char convert[3]={0};

  sprintf(convert,"%01X",_id);

  length_ = 22;
  send_char.resize(length_);
  fill(send_char.begin(),send_char.end(),0);

  send_char[0] = 't';
  send_char[1] = '3';
  send_char[2] = '0';
  send_char[3] = convert[0];
  send_char[4] = '8';
  send_char[5] = 'F';
  send_char[6] = convert[0];
  send_char[7] = '0';
  send_char[8] = '0';

  for(uint8_t i=0;i<6;i++){
    sprintf(convert,"%02X",_data[i]);
    send_char[9+i*2] = convert[0];
    send_char[10+i*2] = convert[1];
  }

  send_char[21] = '\r';

  serial_com_.flushPort();
  serial_com_.writeBuffer(send_char);

#if DEBUG
  std::cout << "send :";
  for (uint8_t i=0;i<length_-1;i++) std::cout << send_char[i] ;
  std::cout << std::endl;
#endif

}

///////////////////////////////
void SeedCommand::writeSerialCommand(uint8_t _id, uint8_t _line, uint8_t *_data)
{
  std::vector<char> send_char;
  char convert[3]={0};

  sprintf(convert,"%01X",_id);

  length_ = 22;
  send_char.resize(length_);
  fill(send_char.begin(),send_char.end(),0);

  send_char[0] = 't';
  send_char[1] = '3';
  send_char[2] = '0';
  send_char[3] = convert[0];
  send_char[4] = '8';
  send_char[5] = 'F';
  send_char[6] = convert[0];

  sprintf(convert,"%02X",_line);
  send_char[7] = convert[0];
  send_char[8] = convert[1];

  for(uint8_t i=0;i<6;i++){
    sprintf(convert,"%02X",_data[i]);
    send_char[9+i*2] = convert[0];
    send_char[10+i*2] = convert[1];
  }

  send_char[21] = '\r';

  serial_com_.flushPort();
  serial_com_.writeBuffer(send_char);

#if DEBUG
  std::cout << "send :";
  for (uint8_t i=0;i<length_-1;i++) std::cout << send_char[i] ;
  std::cout << std::endl;
#endif
}

///////////////////////////////
bool SeedCommand::readSerialCommand(std::vector<uint8_t>& _receive_data, uint16_t _timeout)
{
  std::string data_length = "";
  _receive_data.clear();

  serial_com_.readBufferAsync("\r",_timeout);

  if(serial_com_.receive_buffer_.find("t") != std::string::npos){
      for(size_t i = serial_com_.receive_buffer_.find("t");i<serial_com_.receive_buffer_.size();++i) _receive_data.push_back(serial_com_.receive_buffer_[i]);
      data_length = _receive_data[4];
#if DEBUG
      std::cout << "receive :" ;
      for(auto itr = _receive_data.begin(); itr != _receive_data.end(); ++itr) std::cout << *itr ;
      std::cout << std::endl;
#endif
      if((int)_receive_data.size() < (5 + std::atoi(data_length.c_str()) * 2 + 1)){
#if DEBUG
        std::cout << "size error" << std::endl;
#endif
        return false;
      }
      else return true;
  }
  else{
#if DEBUG
      std::cout << "not CAN commnd" << std::endl;
#endif
      return false;
  }
}

int SeedCommand::str2int(std::string _data)
{
  return std::strtol(_data.c_str(),NULL,16);
}

/*******************
  Fixed Parameters
********************/

///////////////////////////////
void SeedCommand::setTypeNumber(uint8_t _id, const char* _type)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x01;
  send_data_[1] = static_cast<int>(_type[0]);
  send_data_[2] = static_cast<int>(_type[1]);
  send_data_[3] = static_cast<int>(_type[2]);
  send_data_[4] = static_cast<int>(_type[3]);
  send_data_[5] = static_cast<int>(_type[4]);
  writeSerialCommand(_id,send_data_.data());
}


///////////////////////////////
void SeedCommand::setSerialVersion(uint8_t _id, const char* _ver)
{
  uint64_t data,hex_ver;
  fill(send_data_.begin(),send_data_.end(),0);

  sscanf(_ver,"%lx", &data);
  hex_ver = data;

  send_data_[0] = 0x02;
  send_data_[1] = hex_ver >> 32;
  send_data_[2] = hex_ver >> 24;
  send_data_[3] = hex_ver >> 16;
  send_data_[4] = hex_ver >> 8;
  send_data_[5] = hex_ver;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setFirmwareVersion(uint8_t _id, const char* _ver)
{
  uint64_t data,hex_ver;
  fill(send_data_.begin(),send_data_.end(),0);

  sscanf(_ver,"%lx", &data);
  hex_ver = data;

  send_data_[0] = 0x03;
  send_data_[1] = hex_ver >> 32;
  send_data_[2] = hex_ver >> 24;
  send_data_[3] = hex_ver >> 16;
  send_data_[4] = hex_ver >> 8;
  send_data_[5] = hex_ver;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setEditorVersion(uint8_t _id, const char* _ver)
{
  uint64_t data,hex_ver;
  fill(send_data_.begin(),send_data_.end(),0);

  sscanf(_ver,"%lx", &data);
  hex_ver = data;

  send_data_[0] = 0x04;
  send_data_[1] = hex_ver >> 32;
  send_data_[2] = hex_ver >> 24;
  send_data_[3] = hex_ver >> 16;
  send_data_[4] = hex_ver >> 8;
  send_data_[5] = hex_ver;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorAdaptation(uint8_t _id, uint32_t _type, uint16_t _volt)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x05;
  send_data_[1] = _type >>16;
  send_data_[2] = _type >> 8;
  send_data_[3] = _type;
  send_data_[4] = _volt >> 8;
  send_data_[5] = _volt;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorParam(uint8_t _id, uint8_t _mode, uint8_t _feedback)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x06;
  send_data_[1] = _mode;
  send_data_[2] = _feedback;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorCurrentParam(uint8_t _id, uint16_t _driver_max, uint16_t _motor_max, uint8_t _current_conversion)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x07;
  send_data_[1] = _driver_max >> 8;
  send_data_[2] = _driver_max;
  send_data_[3] = _motor_max >> 8;
  send_data_[4] = _motor_max;
  send_data_[5] = _current_conversion;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setCurrentInstantaneous(uint8_t _id, uint16_t _max, uint16_t _time)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x08;
  send_data_[1] = _max >> 8;
  send_data_[2] = _max;
  send_data_[3] = _time >> 8;
  send_data_[4] = _time;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setEncoderParam(uint8_t _id, uint16_t _encoder_pulse, uint16_t _motor_pulse)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x09;
  send_data_[1] = _encoder_pulse >> 8;
  send_data_[2] = _encoder_pulse;
  send_data_[3] = _motor_pulse >> 8;
  send_data_[4] = _motor_pulse;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setDummy(uint8_t _id, uint8_t _cmd)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = _cmd;
  send_data_[1] = 0x00;
  send_data_[2] = 0x00;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

/*******************
  Base Parameters
********************/
///////////////////////////////
void SeedCommand::setIdParam(uint8_t _id, uint8_t _re_id)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x10;
  send_data_[1] = _re_id;
  send_data_[2] = 0x00;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setEmergencyParam(uint8_t _id, uint8_t _mode, uint8_t _io_no, uint8_t _io, uint8_t _reset)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x11;
  send_data_[1] = _mode;
  send_data_[2] = _io_no;
  send_data_[3] = _io;
  send_data_[4] = _reset;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setStopModeParam(uint8_t _id, uint8_t _motor, uint8_t _script)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x12;
  send_data_[1] = _motor;
  send_data_[2] = _script;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setOperationParam(uint8_t _id, uint8_t _auto_run, uint8_t _script, uint8_t _point_go, uint8_t _motor, uint8_t _io)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x13;
  send_data_[1] = _auto_run;
  send_data_[2] = _script;
  send_data_[3] = _point_go;
  send_data_[4] = _motor;
  send_data_[5] = _io;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setOvertravelParam(uint8_t _id, uint8_t _mode, uint8_t _minus, uint8_t _plus, uint8_t _io)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x14;
  send_data_[1] = _mode;
  send_data_[2] = _minus;
  send_data_[3] = _plus;
  send_data_[4] = _io;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setErrorMotionParam(uint8_t _id, uint8_t _temperature, uint8_t _motor, uint8_t _ot, uint8_t _voltage)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x15;
  send_data_[1] = _temperature;
  send_data_[2] = _motor;
  send_data_[3] = _ot;
  send_data_[4] = _voltage;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setResponseParam(uint8_t _id, uint8_t _mode)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x16;
  send_data_[1] = _mode;
  send_data_[2] = 0x00;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setDioParam(uint8_t _id, uint8_t _io0, uint8_t _io1, uint8_t _io2, uint8_t _io3)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x17;
  send_data_[1] = _io0;
  send_data_[2] = _io1;
  send_data_[3] = _io2;
  send_data_[4] = _io3;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setAdParam(uint8_t _id, uint8_t _ad0, uint8_t _ad1, uint8_t _ad2, uint8_t _ad3)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x18;
  send_data_[1] = _ad0;
  send_data_[2] = _ad1;
  send_data_[3] = _ad2;
  send_data_[4] = _ad3;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

/*******************
  Motor Settings
********************/
///////////////////////////////
void SeedCommand::setMotorCurrent(uint8_t _id, uint16_t _max, uint8_t _min, uint16_t _min_time)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x20;
  send_data_[1] = _max >> 8;
  send_data_[2] = _max;
  send_data_[3] = _min;
  send_data_[4] = _min_time >> 8;
  send_data_[5] = _min_time;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorMaxSpeed(uint8_t _id, uint16_t _speed)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x21;
  send_data_[1] = _speed >> 8;
  send_data_[2] = _speed;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorControlParameter1(uint8_t _id, uint8_t _back_surge_a, uint8_t _back_surge_b, uint8_t _back_surge_c, uint8_t _oc, uint8_t _fst)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x22;
  send_data_[1] = _back_surge_a;
  send_data_[2] = _back_surge_b;
  send_data_[3] = _back_surge_c;
  send_data_[4] = _oc;
  send_data_[5] = _fst;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorControlParameter1(uint8_t _id, uint16_t _i_gain, uint8_t _d_gain, uint8_t _switch)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x22;
  send_data_[1] = _i_gain;
  send_data_[2] = _d_gain;
  send_data_[3] = _switch;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setInPosition(uint8_t _id, uint16_t _value)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x23;
  send_data_[1] = _value >> 8;
  send_data_[2] = _value;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setAcDecelerationRate(uint8_t _id, uint16_t _acc, uint16_t _dec)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x24;
  send_data_[1] = _acc >> 8;
  send_data_[2] = _acc;
  send_data_[3] = _dec >> 8;
  send_data_[4] = _dec;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorControlParameter2(uint8_t _id, uint16_t _initial_speed, uint16_t _p_gain, uint8_t _correcting_gain)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x25;
  send_data_[1] = _initial_speed >> 8;
  send_data_[2] = _initial_speed;
  send_data_[3] = _p_gain >> 8;
  send_data_[4] = _p_gain;
  send_data_[5] = _correcting_gain;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setUpperSoftwareLimit(uint8_t _id, int32_t _limit)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x26;
  send_data_[1] = _limit >> 16;
  send_data_[2] = _limit >> 8;
  send_data_[3] = _limit;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setLowerSoftwareLimit(uint8_t _id, int32_t _limit)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x27;
  send_data_[1] = _limit >> 16;
  send_data_[2] = _limit >> 8;
  send_data_[3] = _limit;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorRotation(uint8_t _id, uint8_t _pulse_division, uint8_t _encoder_division, uint8_t _motor_inverted, uint8_t _encoder_inverted, uint8_t _dc_inverted)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x28;
  send_data_[1] = _pulse_division;
  send_data_[2] = _encoder_division;
  send_data_[3] = _motor_inverted;
  send_data_[4] = _encoder_inverted;
  send_data_[5] = _dc_inverted;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorError(uint8_t _id, uint16_t _time, uint32_t _pulse)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x29;
  send_data_[1] = _time >> 8;
  send_data_[2] = _time;
  send_data_[3] = _pulse >> 16;
  send_data_[4] = _pulse >> 8;
  send_data_[5] = _pulse;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setMotorErrorLimit(uint8_t _id, uint8_t _temerature , uint8_t _minimum_voltage, uint8_t _maximum_voltage, uint8_t _current)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x2A;
  send_data_[1] = _temerature;
  send_data_[2] = _minimum_voltage;
  send_data_[3] = _maximum_voltage;
  send_data_[4] = _current;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

/*******************
  Script Settings
********************/
///////////////////////////////
void SeedCommand::setScriptData(uint8_t _id, uint8_t _number , uint8_t _start_line, uint8_t _end_line)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x32;
  send_data_[1] = _number;
  send_data_[2] = _start_line;
  send_data_[3] = _end_line;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::writeScriptLine(uint8_t _id, uint8_t _line , const char* _command)
{
  uint64_t data,hex_command;
  fill(send_data_.begin(),send_data_.end(),0);

  sscanf(_command,"%lx", &data);
  hex_command = data;

  send_data_[0] = hex_command >> 40;
  send_data_[1] = hex_command >> 32;
  send_data_[2] = hex_command >> 24;
  send_data_[3] = hex_command >> 16;
  send_data_[4] = hex_command >> 8;
  send_data_[5] = hex_command;
  writeSerialCommand(_id,_line, send_data_.data());
}

/*******************
  Other
********************/

///////////////////////////////
bool SeedCommand::getLockCode(uint8_t _id, uint8_t _type)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x40;
  send_data_[1] = _type;
  send_data_[2] = _type;
  send_data_[3] = 0xFF;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());

  std::vector<uint8_t> receive_data;
  std::string data = "" ;
  if(!readSerialCommand(receive_data)) return false;
  else{
    lock_code_info_["status"] = std::strtol(data.c_str(),NULL,16);

    for(int i=13;i<17;++i) data += receive_data[i];
    if(_type == 0x1E)lock_code_info_["base_parameters"] = std::strtol(data.c_str(),NULL,16);
    else if(_type == 0x2E)lock_code_info_["motor_settings"] = std::strtol(data.c_str(),NULL,16);
    else if(_type == 0x3E)lock_code_info_["script_data"] = std::strtol(data.c_str(),NULL,16);

    return true;
  }

	//comand_mtx_.unlock();
}

///////////////////////////////
void SeedCommand::setReleaseLock(uint8_t _id, uint8_t _type, uint16_t _code)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = _type;
  send_data_[1] = 0x01;
  send_data_[2] = _code >> 8;
  send_data_[3] = _code;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

//////////////////////////////
std::vector<uint8_t> SeedCommand::getConnectedId()
{
  connected_id_.clear();

  for(int i=1;i<15;++i){
    if(getOperationalInfo(i)[0]) connected_id_.push_back(i);
  }
  return connected_id_;
}

///////////////////////////////
void SeedCommand::writeRom(uint8_t _id, uint8_t _type)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = _type;
  send_data_[1] = 0x00;
  send_data_[2] = 0x00;
  send_data_[3] = 0x00;
  send_data_[4] = 0x00;
  send_data_[5] = 0x00;
  writeSerialCommand(_id,send_data_.data());
}

int SeedCommand::io2int(std::string _parameter)
{
  if(_parameter.find("標準")!= std::string::npos)return 0;
  else if(_parameter.find("プルアップ")!= std::string::npos)return 1;
  else if(_parameter.find("プルダウン")!= std::string::npos)return 2;
  else if(_parameter.find("出力(外部変更可)")!= std::string::npos)return 3;
  else if(_parameter.find("不可")!= std::string::npos)return 4;
  if(_parameter.find("入力")!= std::string::npos)return 5;
  else return 0;
}

/*******************
  get or actuate
********************/

///////////////////////////////
std::array<int, 3> SeedCommand::getPosition(uint8_t _id)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x42;
  send_data_[1] = _id;
  writeSerialCommand(_id,send_data_.data());

  std::vector<uint8_t> receive_data;
  std::string id = "";
  std::string command = "";
  std::string velocity = "" ;
  std::string position = "" ;

  if(!readSerialCommand(receive_data)){
    return {false, 0, 0};
  }
  else{
    id = receive_data[5];
    for(int i=9;i<11;++i) command += receive_data[i];

    if(str2int(command) == send_data_[0] && str2int(id) == _id){
      for(int i=11;i<15;++i) velocity += receive_data[i];

      if(receive_data[15] == 'F') position = "FF";  //in case of negative value
      else position = "";
      for(int i=15;i<21;++i) position += receive_data[i];
      return {true, str2int(velocity), str2int(position)};
    }
    else return {false, 0, 0};
  }
}

///////////////////////////////
std::array<int, 3> SeedCommand::getCurrent(uint8_t _id)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x43;
  send_data_[1] = _id;
  writeSerialCommand(_id,send_data_.data());

  std::vector<uint8_t> receive_data;
  std::string id = "";
  std::string command = "";
  std::string current = "" ;
  std::string position_error = "" ;

  if(!readSerialCommand(receive_data)){
    return {false, 0, 0};
  }
  else{
    id = receive_data[5];
    for(int i=9;i<11;++i) command += receive_data[i];

    if(str2int(command) == send_data_[0] && str2int(id) == _id){
      for(int i=11;i<15;++i) current += receive_data[i];

      if(receive_data[15] == 'F') position_error = "FF";  //in case of negative value
      else position_error ="";
      for(int i=15;i<21;++i) position_error += receive_data[i];

      return {true, str2int(current), str2int(position_error)};
    }
    else return {false, 0, 0};
  }
}

///////////////////////////////
std::array<int, 6> SeedCommand::getOperationalInfo(uint8_t _id)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x44;
  send_data_[1] = _id;
  writeSerialCommand(_id,send_data_.data());

  std::vector<uint8_t> receive_data;
  std::string id = "";
  std::string command = "";
  std::string data = "" ;

  if(!readSerialCommand(receive_data)){
    return {false, 0, 0, 0, 0, 0};
  }
  else{
    id = receive_data[5];
    for(int i=9;i<11;++i) command += receive_data[i];

    if(str2int(command) == send_data_[0] && str2int(id) == _id){
      for(int i=11;i<20;++i) data += receive_data[i];
      operational_info_["status"] = std::strtol(data.substr(0,2).c_str(),NULL,16);
      operational_info_["motor_state"] = std::strtol(data.substr(2,2).c_str(),NULL,16);
      operational_info_["running_script_number"] = std::strtol(data.substr(4,2).c_str(),NULL,16);
      operational_info_["running_script_row"] = std::strtol(data.substr(6,2).c_str(),NULL,16);
      operational_info_["running_point_number"] = std::strtol(data.substr(8,2).c_str(),NULL,16);

      return {true, operational_info_["status"],operational_info_["motor_state"],
        operational_info_["running_script_number"], operational_info_["running_script_row"], operational_info_["running_point_number"]};
    }
    else return {false, 0, 0, 0, 0, 0};
  }
}

///////////////////////////////
void SeedCommand::onServo(uint8_t _id, uint8_t _state)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x50;
  send_data_[1] = _id;
  send_data_[2] = _state;
  writeSerialCommand(_id,send_data_.data());
}
///////////////////////////////
void SeedCommand::stopMotor(uint8_t _id)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x51;
  send_data_[1] = _id;
  writeSerialCommand(_id,send_data_.data());
}
///////////////////////////////
void SeedCommand::runScript(uint8_t _id, uint8_t _script_no)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x5F;
  send_data_[1] = _id;
  send_data_[2] = _script_no;

  if ((_script_no > 0x00) && (_script_no < 0x0F)) writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::waitForScriptEnd(int _number)
{
  int number_of_end = 0;

  while( number_of_end < _number){
    std::vector<uint8_t> receive_data;
    std::string id = "";
    std::string command = "";
    std::string data = "" ;
    if(!readSerialCommand(receive_data,10000)); //10[sec]
    else{
      id = receive_data[5];
      for(int i=9;i<11;++i) command += receive_data[i];
      for(int i=13;i<15;++i) data += receive_data[i];

      if(str2int(data) == 0xFF){
        number_of_end += 1;
        std::cout << "Script of ID " << id << " is the end." << std::endl;
      }
    }
  }
}

///////////////////////////////
void SeedCommand::actuateRelativePositionByTime(uint8_t _id, int16_t _time, int32_t _position)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x61;
  send_data_[1] = _time >> 8;
  send_data_[2] = _time;
  send_data_[3] = _position >> 16;
  send_data_[4] = _position >> 8;
  send_data_[5] = _position;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::actuateRelativePositionBySpeed(uint8_t _id, int16_t _speed, int32_t _position)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x63;
  send_data_[1] = _speed >> 8;
  send_data_[2] = _speed;
  send_data_[3] = _position >> 16;
  send_data_[4] = _position >> 8;
  send_data_[5] = _position;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::actuateAbsolutePositionByTime(uint8_t _id, uint16_t _time, int32_t _position)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x64;
  send_data_[1] = _time >> 8;
  send_data_[2] = _time;
  send_data_[3] = _position >> 16;
  send_data_[4] = _position >> 8;
  send_data_[5] = _position;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::actuateAbsolutePositionBySpeed(uint8_t _id, int16_t _speed, int32_t _position)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x66;
  send_data_[1] = _speed >> 8;
  send_data_[2] = _speed;
  send_data_[3] = _position >> 16;
  send_data_[4] = _position >> 8;
  send_data_[5] = _position;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::actuateContinuousRelativePosition(uint8_t _id, uint16_t _time, int32_t _position)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x67;
  send_data_[1] = _time >> 8;
  send_data_[2] = _time;
  send_data_[3] = _position >> 16;
  send_data_[4] = _position >> 8;
  send_data_[5] = _position;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::actuateContinuousAbsolutePosition(uint8_t _id, uint16_t _time, int32_t _position)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x68;
  send_data_[1] = _time >> 8;
  send_data_[2] = _time;
  send_data_[3] = _position >> 16;
  send_data_[4] = _position >> 8;
  send_data_[5] = _position;
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::actuateBySpeed(uint8_t _id, int16_t _speed)
{
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0x6C;
  if(_speed < 0)
  {
    send_data_[1] = (-1 * _speed) >> 8;
    send_data_[2] = (-1 * _speed);
    send_data_[3] = 1;
  }
  else
  {
    send_data_[1] = _speed >> 8;
    send_data_[2] = _speed;
    send_data_[3] = 0;
  }
  writeSerialCommand(_id,send_data_.data());
}

///////////////////////////////
void SeedCommand::setPosition(uint8_t _id, uint8_t _position, uint8_t _state)
{
  fill(send_data_.begin(),send_data_.end(),0);
  send_data_[0] = 0x6F;
  send_data_[1] = _position;
  send_data_[2] = _state;
  writeSerialCommand(_id,send_data_.data());
}

////////////////////////////////

