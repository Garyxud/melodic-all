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

#ifndef PSEN_SCAN_ROS_SCANNER_NODE_H
#define PSEN_SCAN_ROS_SCANNER_NODE_H

#include "psen_scan/scanner.h"
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <ros/ros.h>

namespace psen_scan
{
/**
 * @brief Class implements a ROS-Node for the PSENscan safety laser scanner
 *
 */
class ROSScannerNode
{
public:
  ROSScannerNode(ros::NodeHandle& nh,
                 const std::string& topic,
                 const std::string& frame_id,
                 const uint16_t& skip,
                 const Degree& x_axis_rotation,
                 std::unique_ptr<vScanner> scanner);
  sensor_msgs::LaserScan buildRosMessage(const LaserScan& laserscan);
  void processingLoop();

private:
  ros::NodeHandle nh_;                /**< ROS Node handler*/
  ros::Publisher pub_;                /**< ROS message publisher*/
  std::string frame_id_;              /**< Defines the name of the frame_id. Default is scanner.*/
  uint16_t skip_;                     /**< Skip certain number of frames. Reduces publish rate. */
  std::unique_ptr<vScanner> scanner_; /**< Points to an instance of the Scanner class.*/
  Degree x_axis_rotation_;            /**< X-axis rotation.*/
};
}  // namespace psen_scan

#endif  // PSEN_SCAN_ROS_SCANNER_NODE_H