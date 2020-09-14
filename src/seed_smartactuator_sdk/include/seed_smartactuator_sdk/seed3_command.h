#ifndef SEED_COMMAND_H_
#define SEED_COMMAND_H_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <unordered_map>

using namespace boost::asio;

namespace seed
{
  namespace controller
  {
    class SerialCommunication
    {
    public:
      SerialCommunication();
      ~SerialCommunication();

      bool openPort(std::string _port, unsigned int _baud_rate);
      void closePort();
      void writeBuffer(std::vector<char>& _send_data);
      std::string readBuffer(uint16_t _wait_time=0);
      void flushPort();

      void readBufferAsync(std::string _delim, uint16_t _timeout=50);
      void onReceive(const boost::system::error_code& _error, size_t _bytes_transferred);
      void can_receive(std::vector<uint8_t>& _receive_data);
      //--------- Receive timer event ------
      void onTimer(const boost::system::error_code& _error);

      std::string receive_buffer_;

    private:
      io_service io_;
      serial_port serial_;
      deadline_timer timer_;

      boost::mutex serial_mtx_;

      bool is_canceled_;
      boost::asio::streambuf stream_buffer_;

    };

    class SeedCommand{
    public:
      SeedCommand();
      ~SeedCommand();

      bool is_open_;

      bool openPort(std::string _port, unsigned int _baud_rate);
      void closePort();
      void openCom();
      void closeCom();
      void writeSerialCommand(uint8_t _id, uint8_t *_data);
      void writeSerialCommand(uint8_t _id, uint8_t _line, uint8_t *_data);
      bool readSerialCommand(std::vector<uint8_t>& _receive_data, uint16_t _timeout=50);

      //seed_information
      std::unordered_map<std::string, int32_t> seed_info_;
      std::unordered_map<std::string, int32_t> operational_info_;
      std::unordered_map<std::string, int16_t> lock_code_info_;
      std::vector<uint8_t> connected_id_;

      int str2int(std::string _data);

      //Fixed Parameters---------
      void setTypeNumber(uint8_t _id, const char* _type);
      void setEditorVersion(uint8_t _id, const char* _ver);
      void setMotorCurrentParam(uint8_t _id, uint16_t _driver_max, uint16_t _motor_max, uint8_t _current_conversion);
      void setDummy(uint8_t _id, uint8_t _cmd);
      void setSerialVersion(uint8_t _id, const char* _ver);
      void setMotorAdaptation(uint8_t _id, uint32_t _type, uint16_t _volt);
      void setCurrentInstantaneous(uint8_t _id, uint16_t _max, uint16_t _time);
      void setFirmwareVersion(uint8_t _id, const char* _ver);
      void setMotorParam(uint8_t _id, uint8_t _mode, uint8_t _feedback);
      void setEncoderParam(uint8_t _id, uint16_t _encoder_pulse, uint16_t _motor_pulse);

      //Base Parameters---------
      void setIdParam(uint8_t _id, uint8_t _re_id);
      void setEmergencyParam(uint8_t _id, uint8_t _mode, uint8_t _io_no, uint8_t _io, uint8_t _reset);
      void setStopModeParam(uint8_t _id, uint8_t _motor, uint8_t _script);
      void setOperationParam(uint8_t _id, uint8_t _auto_run, uint8_t _script, uint8_t _point_go, uint8_t _motor, uint8_t _io);
      void setOvertravelParam(uint8_t _id, uint8_t _mode, uint8_t _minus, uint8_t _plus, uint8_t _io);
      void setErrorMotionParam(uint8_t _id, uint8_t _temperature, uint8_t _motor, uint8_t _ot, uint8_t _voltage);
      void setResponseParam(uint8_t _id, uint8_t _mode);
      void setDioParam(uint8_t _id, uint8_t _io0, uint8_t _io1, uint8_t _io2, uint8_t _io3);
      void setAdParam(uint8_t _id, uint8_t _ad0, uint8_t _ad1, uint8_t _ad2, uint8_t _ad3);

      //Motor Settings---------
      void setMotorCurrent(uint8_t _id, uint16_t _max, uint8_t _min, uint16_t _min_time);
      void setMotorMaxSpeed(uint8_t _id, uint16_t _speed);
      void setMotorControlParameter1(uint8_t _id, uint8_t _back_surge_a, uint8_t _back_surge_b, uint8_t _back_surge_c, uint8_t _oc, uint8_t _fst);
      void setMotorControlParameter1(uint8_t _id, uint16_t _i_gain, uint8_t _d_gain, uint8_t _switch);
      void setInPosition(uint8_t _id, uint16_t _value);
      void setAcDecelerationRate(uint8_t _id, uint16_t _acc, uint16_t _dec);
      void setMotorControlParameter2(uint8_t _id, uint16_t _initial_speed, uint16_t _p_gain, uint8_t _correcting_gain);
      void setUpperSoftwareLimit(uint8_t _id, int32_t _limit);
      void setLowerSoftwareLimit(uint8_t _id, int32_t _limit);
      void setMotorRotation(uint8_t _id, uint8_t _pulse_division, uint8_t _encoder_division, uint8_t _motor_inverted, uint8_t _encoder_inverted, uint8_t _dc_inverted);
      void setMotorError(uint8_t _id, uint16_t _time, uint32_t _pulse);
      void setMotorErrorLimit(uint8_t _id, uint8_t _temerature , uint8_t _minimum_voltage, uint8_t _maximum_voltage, uint8_t _current);

      //Script Settings-----
      void setScriptData(uint8_t _id, uint8_t _number , uint8_t _start_line, uint8_t _end_line);
      void writeScriptLine(uint8_t _id, uint8_t _line , const char* _command);

      //Other-------
      bool getLockCode(uint8_t _id, uint8_t _mode);
      void setReleaseLock(uint8_t _id, uint8_t _type, uint16_t _code);
      std::vector<uint8_t> getConnectedId();
      void writeRom(uint8_t _id, uint8_t type);
      int io2int(std::string _parameter);

      //run motor command------
      std::array<int, 3> getPosition(uint8_t _id);
      std::array<int, 3> getCurrent(uint8_t _id);
      std::array<int, 6> getOperationalInfo(uint8_t _id);

      void onServo(uint8_t _id, uint8_t _state);
      void stopMotor(uint8_t _id);
      void runScript(uint8_t _id, uint8_t _script_no);
      void waitForScriptEnd(int _number);
      void actuateRelativePositionByTime(uint8_t _id, int16_t _time, int32_t _position);
      void actuateRelativePositionBySpeed(uint8_t _id, int16_t _speed, int32_t _position);
      void actuateAbsolutePositionByTime(uint8_t _id, uint16_t _time, int32_t _position);
      void actuateAbsolutePositionBySpeed(uint8_t _id, int16_t _speed, int32_t _position);
      void actuateContinuousRelativePosition(uint8_t _id, uint16_t _time, int32_t _position);
      void actuateContinuousAbsolutePosition(uint8_t _id, uint16_t _time, int32_t _position);
      void actuateBySpeed(uint8_t _id, int16_t _speed);

      void setPosition(uint8_t _id, uint8_t _position, uint8_t _state);

    private:
      //Value
      unsigned int check_sum_,count_,length_;
      std::vector<uint8_t> send_data_;

    protected:
      SerialCommunication serial_com_;
    };

  } //end namespce controller
} //end namespace seed

#endif
