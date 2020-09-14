// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <psen_scan/ros_parameter_handler.h>
#include "psen_scan/get_ros_parameter_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/decrypt_password_exception.h"
#include <psen_scan/default_parameters.h>
#include <psen_scan/scanner_data.h>
#include <arpa/inet.h>
#include <algorithm>

namespace psen_scan
{
/**
 * @brief Construct a new Ros Parameter Handler:: Ros Parameter Handler object
 *
 * Gets all Parameters from Parameter Server.
 *
 * @param nh Nodehandle from which parameters will be fetched
 *
 * @throws PSENScanFatalException
 */
RosParameterHandler::RosParameterHandler(const ros::NodeHandle& nh)
  : nh_(nh)
  , host_ip_()
  , host_udp_port_()
  , frame_id_(DEFAULT_FRAME_ID)
  , skip_(DEFAULT_SKIP)
  , angle_start_(DEFAULT_ANGLE_START)
  , angle_end_(DEFAULT_ANGLE_END)
  , x_axis_rotation_(DEFAULT_X_AXIS_ROTATION)
{
  updateAllParamsFromParamServer();
}

/**
 * @brief Update all Parameters from ROS Parameter Server
 *
 */
void RosParameterHandler::updateAllParamsFromParamServer()
{
  // required parameters first
  // update parameter password
  try
  {
    getRequiredParamFromParamServer<std::string>("password", password_);
    try
    {
      password_ = decryptPassword(password_);
    }
    catch (const DecryptPasswordException& e)
    {
      throw PSENScanFatalException("Invalid Password: " + std::string(e.what()));
    }
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter password
  // update parameter host_ip
  try
  {
    std::string host_ip;
    getRequiredParamFromParamServer<std::string>("host_ip", host_ip);
    in_addr_t ip_addr = inet_network(host_ip.c_str());
    if (static_cast<in_addr_t>(-1) == ip_addr)
    {
      throw PSENScanFatalException("Host IP address conversion failed!");
    }
    host_ip_ = htobe32(ip_addr);
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter host_ip
  // update parameter host_udp_port
  try
  {
    int host_udp_port;
    getRequiredParamFromParamServer<int>("host_udp_port", host_udp_port);
    if (host_udp_port < 0)
    {
      throw PSENScanFatalException("Parameter host_udp_port may not be negative!");
    }
    if (host_udp_port > 65535)
    {
      throw PSENScanFatalException("Parameter host_udp_port too large!");
    }
    host_udp_port_ = htole32(static_cast<uint32_t>(host_udp_port));
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter host_udp_port
  // update parameter sensor_ip
  try
  {
    std::string sensor_ip;
    getRequiredParamFromParamServer<std::string>("sensor_ip", sensor_ip);
    in_addr_t ip_addr = inet_network(sensor_ip.c_str());
    if (static_cast<in_addr_t>(-1) == ip_addr)
    {
      throw PSENScanFatalException("Sensor IP address conversion failed!");
    }
    sensor_ip_ = sensor_ip;
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter sensor_ip

  // non-required parameters last
  // update parameter frame_id
  try
  {
    getOptionalParamFromParamServer<std::string>("frame_id", frame_id_);
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter frame_id
  // update parameter skip
  try
  {
    int skip;
    if (getOptionalParamFromParamServer<int>("skip", skip))
    {
      if (skip < 0)
      {
        throw PSENScanFatalException("Parameter skip may not be negative!");
      }
      skip_ = static_cast<uint16_t>(skip);
    }
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter skip
  // update parameter angle_start
  try
  {
    float angle_start;
    if (getOptionalParamFromParamServer<float>("angle_start", angle_start))
    {
      if (PSENscanInternalAngle(Degree(angle_start)) < MIN_SCAN_ANGLE)
      {
        throw PSENScanFatalException("Parameter angle_start may not be negative!");
      }
      if (PSENscanInternalAngle(Degree(angle_start)) > MAX_SCAN_ANGLE)
      {
        throw PSENScanFatalException("Parameter angle_start too large!");
      }
      angle_start_ = PSENscanInternalAngle(Degree(angle_start));
    }
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter angle_start
  // update parameter angle_end
  try
  {
    float angle_end;
    if (getOptionalParamFromParamServer<float>("angle_end", angle_end))
    {
      if (PSENscanInternalAngle(Degree(angle_end)) < MIN_SCAN_ANGLE)
      {
        throw PSENScanFatalException("Parameter angle_end may not be negative!");
      }
      if (PSENscanInternalAngle(Degree(angle_end)) > MAX_SCAN_ANGLE)
      {
        throw PSENScanFatalException("Parameter angle_end too large!");
      }
      angle_end_ = PSENscanInternalAngle(Degree(angle_end));
    }
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter angle_end
  // update parameter x_axis_rotation
  try
  {
    double x_axis_rotation;
    if (getOptionalParamFromParamServer<double>("x_axis_rotation", x_axis_rotation))
    {
      x_axis_rotation_ = Degree(x_axis_rotation);
      if ((x_axis_rotation_ > MAX_X_AXIS_ROTATION) || (x_axis_rotation_ < MIN_X_AXIS_ROTATION))
      {
        throw PSENScanFatalException("Parameter x_axis_rotation is out of range. [-360.0 .. 360.0]");
      }
    }
  }
  catch (const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    throw PSENScanFatalException("Reading of required parameter failed!");
  }  // update parameter x_axis_rotation
}

/**
 * @brief Gets one required ROS-parameter from parameter server
 *
 * @tparam T Type of parameter to fetch
 * @param key Key for the parameter on parameter-server
 *
 * @throws GetROSParameterException
 */
template <class T>
void RosParameterHandler::getRequiredParamFromParamServer(const std::string& key, T& param)
{
  if (!nh_.hasParam(key))
  {
    throw GetROSParameterException("Parameter " + key + " doesn't exist on parameter server.");
  }

  if (!nh_.getParam(key, param))
  {
    throw GetROSParameterException("Parameter " + key + " has wrong datatype on parameter server.");
  }
  return;
}

/**
 * @brief Gets one optional ROS-parameter from parameter server
 *
 * @tparam T Type of parameter to fetch
 * @param key Key for the parameter on parameter-server
 * @return true for sucess and false if the parameter does not exist
 *
 * @throws GetROSParameterException
 */
template <class T>
bool RosParameterHandler::getOptionalParamFromParamServer(const std::string& key, T& param)
{
  if (!nh_.hasParam(key))
  {
    ROS_WARN_STREAM("Parameter " + key + " doesn't exist on parameter server. Proceeding with default configuration.");
    return false;
  }
  if (!nh_.getParam(key, param))
  {
    throw GetROSParameterException("Parameter " + key + " has wrong datatype on parameter server.");
  }
  return true;
}

/**
 * @brief Getter method for password_
 *
 * @return std::string
 */
std::string RosParameterHandler::getPassword() const
{
  return password_;
}

/**
 * @brief Getter method for host_ip_
 *
 * @return uint32_t
 */
uint32_t RosParameterHandler::getHostIP() const
{
  return host_ip_;
}

/**
 * @brief Getter method for host_udp_port_
 *
 * @return uint32_t
 */
uint32_t RosParameterHandler::getHostUDPPort() const
{
  return host_udp_port_;
}

/**
 * @brief Getter Method for sensor_ip_
 *
 * @return std::string
 */
std::string RosParameterHandler::getSensorIP() const
{
  return sensor_ip_;
}

/**
 * @brief Getter method for frame_id_
 *
 * @return std::string
 */
std::string RosParameterHandler::getFrameID() const
{
  return frame_id_;
}

/**
 * @brief Getter method for skip_
 *
 * @return uint16_t
 */
uint16_t RosParameterHandler::getSkip() const
{
  return skip_;
}

/**
 * @brief Getter Method for angle_start_
 *
 * @return uint16_t
 */
PSENscanInternalAngle RosParameterHandler::getAngleStart() const
{
  return angle_start_;
}

/**
 * @brief Getter Method for angle_end_
 *
 * @return uint16_t
 */
PSENscanInternalAngle RosParameterHandler::getAngleEnd() const
{
  return angle_end_;
}

/**
 * @brief Getter Method for x_axis_rotation_
 *
 * @return double
 */
Degree RosParameterHandler::getXAxisRotation() const
{
  return x_axis_rotation_;
}

/**
 * @brief Decrypt password
 *
 * @param encrypted_password Encrypted Password
 * @return Decrypted Password
 *
 * @throws DecryptPasswordException
 */
std::string RosParameterHandler::decryptPassword(const std::string& encrypted_password)
{
  const int encrypted_char_len = 2;
  const int encrypted_char_base = 16;
  const int addition_coeff = 100;  // arbitrary
  const int number_of_ascii_chars = 256;
  const int encryption_xor_key = 0xCD;  // arbitrary

  std::string decrypted_password = "";

  std::string encrypted_pw_temp = encrypted_password;

  encrypted_pw_temp.erase(std::remove(encrypted_pw_temp.begin(), encrypted_pw_temp.end(), ' '),
                          encrypted_pw_temp.end());

  if (encrypted_pw_temp.length() % 2 != 0)
  {
    throw DecryptPasswordException("Password length must be even!");
  }

  for (unsigned int i = 0; i < encrypted_pw_temp.length(); i += encrypted_char_len)
  {
    char c;
    try
    {
      c = (std::stoi(encrypted_pw_temp.substr(i, encrypted_char_len), nullptr, encrypted_char_base) -
           i * addition_coeff / encrypted_char_len) %
              number_of_ascii_chars ^
          encryption_xor_key;
    }
    catch (const std::invalid_argument& e)
    {
      throw DecryptPasswordException(e.what());
    }

    if (c < 32)
    {
      throw DecryptPasswordException("Control characters not allowed!");
    }

    decrypted_password += c;
  }

  return decrypted_password;
}

}  // namespace psen_scan