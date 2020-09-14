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

#ifndef PSEN_SCAN_ROS_PARAMETER_HANDLER_H
#define PSEN_SCAN_ROS_PARAMETER_HANDLER_H

#include <ros/ros.h>
#include <psen_scan/psen_scan_internal_angle.h>

namespace psen_scan
{
/**
 * @brief Class for getting ROS-Parameters from the parameter-server
 *
 */
class RosParameterHandler
{
public:
  RosParameterHandler(const ros::NodeHandle& nh);
  void updateAllParamsFromParamServer();
  template <class T>
  void getRequiredParamFromParamServer(const std::string& key, T& param);
  template <class T>
  bool getOptionalParamFromParamServer(const std::string& key, T& param);
  std::string getPassword() const;
  uint32_t getHostIP() const;
  uint32_t getHostUDPPort() const;
  std::string getSensorIP() const;
  std::string getFrameID() const;
  uint16_t getSkip() const;
  PSENscanInternalAngle getAngleStart() const;
  PSENscanInternalAngle getAngleEnd() const;
  Degree getXAxisRotation() const;

private:
  ros::NodeHandle const nh_;          /**< Nodehandle through which parameters are fetched */
  std::string password_;              /**< Password for Laserscanner */
  uint32_t host_ip_;                  /**< IP-Adress of host machine */
  uint32_t host_udp_port_;            /**< UDP Port on which packets from Laserscanner should be received */
  std::string sensor_ip_;             /**< IP-Adress of Safety laser scanner */
  std::string frame_id_;              /**< ROS Frame ID */
  uint16_t skip_;                     /**< How many incoming frames should be skipped (reduces publish rate) */
  PSENscanInternalAngle angle_start_; /**< Start angle of measurement */
  PSENscanInternalAngle angle_end_;   /**< End angle of measurement */
  Degree x_axis_rotation_;            /**< Rotation of x-axis arround the center */

public:
  static std::string decryptPassword(const std::string& encrypted_password);
};
}  // namespace psen_scan

#endif  // PSEN_SCAN_ROS_PARAMETER_HANDLER_H