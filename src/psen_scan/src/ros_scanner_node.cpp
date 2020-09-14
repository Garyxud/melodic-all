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

#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/coherent_monitoring_frames_exception.h"
#include "psen_scan/diagnostic_information_exception.h"
#include "psen_scan/parse_monitoring_frame_exception.h"
#include "psen_scan/build_ros_message_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include <psen_scan/scanner_data.h>

namespace psen_scan
{
/**
 * @brief Construct a new ROSScannerNode::ROSScannerNode object
 *
 * @param nh node handle for the ROS node
 * @param topic name of the ROS topic
 * @param frame_id name of the frame id
 * @param skip skip certain number of frames, reduces publish rate
 * @param scanner pointer ot an instance of the class Scanner
 */
ROSScannerNode::ROSScannerNode(ros::NodeHandle& nh,
                               const std::string& topic,
                               const std::string& frame_id,
                               const uint16_t& skip,
                               const Degree& x_axis_rotation,
                               std::unique_ptr<vScanner> scanner)
  : nh_(nh), frame_id_(frame_id), skip_(skip), scanner_(std::move(scanner)), x_axis_rotation_(x_axis_rotation)
{
  if (!scanner_)
  {
    throw PSENScanFatalException("Nullpointer isn't a valid argument!");
  }
  pub_ = nh_.advertise<sensor_msgs::LaserScan>(topic, 1);
}

/**
 * @brief Convert angle in deg to rad
 *
 * @param deg angle in degree
 *
 * @return double, angle in rad
 *
 */
double degToRad(const Degree& deg)
{
  return static_cast<double>(deg) * M_PI / 180.0;
}

/**
 * @brief Creates a ROS message from an LaserScan, which contains one scanning round.
 *
 * @param laserscan
 *
 * @return sensor_msgs::LaserScan, ROS message, ready to be published
 *
 * @throws BuildROSMessageException
 *
 */
sensor_msgs::LaserScan ROSScannerNode::buildRosMessage(const LaserScan& laserscan)
{
  if (laserscan.resolution_ == PSENscanInternalAngle(0))
  {
    throw BuildROSMessageException("Resolution cannot be 0!");
  }
  if (laserscan.min_scan_angle_ >= laserscan.max_scan_angle_)
  {
    throw BuildROSMessageException("Attention: Start angle has to be smaller than end angle!");
  }
  uint16_t expected_size =
      static_cast<int>(laserscan.max_scan_angle_ - laserscan.min_scan_angle_) / static_cast<int>(laserscan.resolution_);
  if (expected_size != laserscan.measures_.size())
  {
    throw BuildROSMessageException("Calculated number of scans doesn't match actual number of scans!");
  }

  sensor_msgs::LaserScan ros_message;
  ros_message.header.stamp = ros::Time::now();
  ros_message.header.frame_id = frame_id_;
  ros_message.angle_min = degToRad(Degree(laserscan.min_scan_angle_) - x_axis_rotation_);
  ros_message.angle_max = degToRad(Degree(laserscan.max_scan_angle_) - x_axis_rotation_);
  ros_message.angle_increment = degToRad(Degree(laserscan.resolution_));
  ros_message.time_increment = SCAN_TIME / NUMBER_OF_SAMPLES_FULL_SCAN_MASTER;
  ros_message.scan_time = SCAN_TIME;
  ros_message.range_min = 0;
  ros_message.range_max = 10;
  ros_message.ranges.insert(ros_message.ranges.end(),
                            laserscan.measures_.rbegin(),
                            laserscan.measures_.rend());  // reverse order
  std::transform(ros_message.ranges.begin(), ros_message.ranges.end(), ros_message.ranges.begin(), [](float f) {
    return f * 0.001;
  });

  return ros_message;
}
/**
 * @brief endless loop for processing incoming UDP data from the laser scanner
 *
 */
void ROSScannerNode::processingLoop()
{
  uint16_t skip_counter = 0;

  scanner_->start();
  while (ros::ok())
  {
    try
    {
      LaserScan complete_scan = scanner_->getCompleteScan();
      if (skip_counter == skip_)
      {
        pub_.publish(buildRosMessage(complete_scan));
      }
    }
    catch (const CoherentMonitoringFramesException& e)
    {
      ROS_WARN_STREAM("Could not build a coherent message: " << e.what() << " Skipping this message.");
    }
    catch (const ParseMonitoringFrameException& e)
    {
      ROS_FATAL_STREAM("Fatal error occured: " << e.what());
    }
    catch (const DiagnosticInformationException& e)
    {
      ROS_FATAL_STREAM("Fatal error occured: " << e.what());
      // throw PSENScanFatalException("");
    }
    catch (const BuildROSMessageException& e)
    {
      ROS_WARN_STREAM("Could not build ROS message: " << e.what() << " Skipping this message.");
    }

    if (++skip_counter > skip_)
    {
      skip_counter = 0;
    }
  }
  scanner_->stop();
}

}  // namespace psen_scan
