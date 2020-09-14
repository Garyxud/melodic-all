/**
 *
 *  \file
 *  \brief      Main entry point for UM6 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#include <string>

#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "um6/comms.h"
#include "um6/registers.h"
#include "um6/Reset.h"

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = UM6_TEMPERATURE;

/**
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in UM6 registers.
 */
template<typename RegT>
void configureVector3(um6::Comms* sensor, const um6::Accessor<RegT>& reg,
                      std::string param, std::string human_name)
{
  if (reg.length != 3)
  {
    throw std::logic_error("configureVector3 may only be used with 3-field registers!");
  }

  if (ros::param::has(param))
  {
    double x, y, z;
    ros::param::get(param + "/x", x);
    ros::param::get(param + "/y", y);
    ros::param::get(param + "/z", z);
    ROS_INFO_STREAM("Configuring " << human_name << " to ("
                    << x << ", " << y << ", " << z << ")");
    reg.set_scaled(0, x);
    reg.set_scaled(1, y);
    reg.set_scaled(2, z);
    if (sensor->sendWaitAck(reg))
    {
      throw std::runtime_error("Unable to configure vector.");
    }
  }
}

/**
 * Function generalizes the process of commanding the UM6 via one of its command
 * registers.
 */
template<typename RegT>
void sendCommand(um6::Comms* sensor, const um6::Accessor<RegT>& reg, std::string human_name)
{
  ROS_INFO_STREAM("Sending command: " << human_name);
  if (!sensor->sendWaitAck(reg))
  {
    throw std::runtime_error("Command to device failed.");
  }
}


/**
 * Send configuration messages to the UM6, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(um6::Comms* sensor, ros::NodeHandle *private_nh)
{
  um6::Registers r;

  // Enable outputs we need.
  const uint8_t UM6_BAUD_115200 = 0x5;
  uint32_t comm_reg = UM6_BROADCAST_ENABLED |
                      UM6_GYROS_PROC_ENABLED | UM6_ACCELS_PROC_ENABLED | UM6_MAG_PROC_ENABLED |
                      UM6_QUAT_ENABLED | UM6_EULER_ENABLED | UM6_COV_ENABLED | UM6_TEMPERATURE_ENABLED |
                      UM6_BAUD_115200 << UM6_BAUD_START_BIT;
  // set the broadcast rate of the device
  int rate;
  private_nh->param<int>("update_rate", rate, 20);
  if (rate < 20 || rate > 300)
  {
    ROS_WARN("Potentially unsupported update rate of %d", rate);
  }
  // converting from desired rate to broadcast_rate as defined in UM6 datasheet
  uint32_t rate_bits = (uint32_t) ((rate-20)*255.0/280.0);
  ROS_INFO("Setting update rate to %uHz", rate);
  comm_reg |= (rate_bits & UM6_SERIAL_RATE_MASK);
  r.communication.set(0, comm_reg);
  if (!sensor->sendWaitAck(r.communication))
  {
    throw std::runtime_error("Unable to set communication register.");
  }

  // Optionally disable mag and accel updates in the sensor's EKF.
  bool mag_updates, accel_updates;
  private_nh->param<bool>("mag_updates", mag_updates, true);
  private_nh->param<bool>("accel_updates", accel_updates, true);
  uint32_t misc_config_reg = UM6_QUAT_ESTIMATE_ENABLED;
  if (mag_updates)
  {
    misc_config_reg |= UM6_MAG_UPDATE_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding magnetometer updates from EKF.");
  }
  if (accel_updates)
  {
    misc_config_reg |= UM6_ACCEL_UPDATE_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding accelerometer updates from EKF.");
  }
  r.misc_config.set(0, misc_config_reg);
  if (!sensor->sendWaitAck(r.misc_config))
  {
    throw std::runtime_error("Unable to set misc config register.");
  }

  // Optionally disable the gyro reset on startup. A user might choose to do this
  // if there's an external process which can ascertain when the vehicle is stationary
  // and periodically call the /reset service.
  bool zero_gyros;
  private_nh->param<bool>("zero_gyros", zero_gyros, true);
  if (zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");

  // Configurable vectors.
  configureVector3(sensor, r.mag_ref, "~mag_ref", "magnetic reference vector");
  configureVector3(sensor, r.accel_ref, "~accel_ref", "accelerometer reference vector");
  configureVector3(sensor, r.mag_bias, "~mag_bias", "magnetic bias vector");
  configureVector3(sensor, r.accel_bias, "~accel_bias", "accelerometer bias vector");
  configureVector3(sensor, r.gyro_bias, "~gyro_bias", "gyroscope bias vector");
}


bool handleResetService(um6::Comms* sensor,
                        const um6::Reset::Request& req, const um6::Reset::Response& resp)
{
  um6::Registers r;
  if (req.zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  if (req.reset_ekf) sendCommand(sensor, r.cmd_reset_ekf, "reset EKF");
  if (req.set_mag_ref) sendCommand(sensor, r.cmd_set_mag_ref, "set magnetometer reference");
  if (req.set_accel_ref) sendCommand(sensor, r.cmd_set_accel_ref, "set accelerometer reference");
  return true;
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(um6::Registers& r, ros::NodeHandle* imu_nh, sensor_msgs::Imu& imu_msg,
    bool tf_ned_to_enu, bool use_magnetic_field_msg)
{
  static ros::Publisher imu_pub = imu_nh->advertise<sensor_msgs::Imu>("data", 1, false);
  static ros::Publisher mag_pub;
  if (use_magnetic_field_msg)
  {
    mag_pub = imu_nh->advertise<sensor_msgs::MagneticField>("mag", 1, false);
  }
  else
  {
    mag_pub = imu_nh->advertise<geometry_msgs::Vector3Stamped>("mag", 1, false);
  }
  static ros::Publisher rpy_pub = imu_nh->advertise<geometry_msgs::Vector3Stamped>("rpy", 1, false);
  static ros::Publisher temp_pub = imu_nh->advertise<std_msgs::Float32>("temperature", 1, false);

  if (imu_pub.getNumSubscribers() > 0)
  {
    // IMU reports a 4x4 wxyz covariance, ROS requires only 3x3 xyz.
    // NED -> ENU conversion req'd?
    imu_msg.orientation_covariance[0] = r.covariance.get_scaled(5);
    imu_msg.orientation_covariance[1] = r.covariance.get_scaled(6);
    imu_msg.orientation_covariance[2] = r.covariance.get_scaled(7);
    imu_msg.orientation_covariance[3] = r.covariance.get_scaled(9);
    imu_msg.orientation_covariance[4] = r.covariance.get_scaled(10);
    imu_msg.orientation_covariance[5] = r.covariance.get_scaled(11);
    imu_msg.orientation_covariance[6] = r.covariance.get_scaled(13);
    imu_msg.orientation_covariance[7] = r.covariance.get_scaled(14);
    imu_msg.orientation_covariance[8] = r.covariance.get_scaled(15);

    // NED -> ENU conversion (x = y, y = x, z = -z)
    if (tf_ned_to_enu)
    {
      imu_msg.orientation.x = r.quat.get_scaled(2);
      imu_msg.orientation.y = r.quat.get_scaled(1);
      imu_msg.orientation.z = -r.quat.get_scaled(3);
      imu_msg.orientation.w = r.quat.get_scaled(0);

      imu_msg.angular_velocity.x = r.gyro.get_scaled(1);
      imu_msg.angular_velocity.y = r.gyro.get_scaled(0);
      imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

      imu_msg.linear_acceleration.x = r.accel.get_scaled(1);
      imu_msg.linear_acceleration.y = r.accel.get_scaled(0);
      imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);
    }
    else
    {
      imu_msg.orientation.w = r.quat.get_scaled(0);
      imu_msg.orientation.x = r.quat.get_scaled(1);
      imu_msg.orientation.y = r.quat.get_scaled(2);
      imu_msg.orientation.z = r.quat.get_scaled(3);

      imu_msg.angular_velocity.x = r.gyro.get_scaled(0);
      imu_msg.angular_velocity.y = r.gyro.get_scaled(1);
      imu_msg.angular_velocity.z = r.gyro.get_scaled(2);

      imu_msg.linear_acceleration.x = r.accel.get_scaled(0);
      imu_msg.linear_acceleration.y = r.accel.get_scaled(1);
      imu_msg.linear_acceleration.z = r.accel.get_scaled(2);
    }

    imu_pub.publish(imu_msg);
  }

  if (mag_pub.getNumSubscribers() > 0)
  {
    if (use_magnetic_field_msg)
    {
      sensor_msgs::MagneticField mag_msg;
      mag_msg.header = imu_msg.header;

      if (tf_ned_to_enu)
      {
        mag_msg.magnetic_field.x = r.mag.get_scaled(1);
        mag_msg.magnetic_field.y = r.mag.get_scaled(0);
        mag_msg.magnetic_field.z = -r.mag.get_scaled(2);
      }
      else
      {
        mag_msg.magnetic_field.x = r.mag.get_scaled(0);
        mag_msg.magnetic_field.y = r.mag.get_scaled(1);
        mag_msg.magnetic_field.z = r.mag.get_scaled(2);
      }

      mag_pub.publish(mag_msg);
    }
    else
    {
      geometry_msgs::Vector3Stamped mag_msg;
      mag_msg.header = imu_msg.header;

      if (tf_ned_to_enu)
      {
        mag_msg.vector.x = r.mag.get_scaled(1);
        mag_msg.vector.y = r.mag.get_scaled(0);
        mag_msg.vector.z = -r.mag.get_scaled(2);
      }
      else
      {
        mag_msg.vector.x = r.mag.get_scaled(0);
        mag_msg.vector.y = r.mag.get_scaled(1);
        mag_msg.vector.z = r.mag.get_scaled(2);
      }

      mag_pub.publish(mag_msg);
    }
  }

  if (rpy_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = imu_msg.header;

    if (tf_ned_to_enu)
    {
      rpy_msg.vector.x = r.euler.get_scaled(1);
      rpy_msg.vector.y = r.euler.get_scaled(0);
      rpy_msg.vector.z = -r.euler.get_scaled(2);
    }
    else
    {
      rpy_msg.vector.x = r.euler.get_scaled(0);
      rpy_msg.vector.y = r.euler.get_scaled(1);
      rpy_msg.vector.z = r.euler.get_scaled(2);
    }

    rpy_pub.publish(rpy_msg);
  }

  if (temp_pub.getNumSubscribers() > 0)
  {
    std_msgs::Float32 temp_msg;
    temp_msg.data = r.temperature.get_scaled(0);
    temp_pub.publish(temp_msg);
  }
}

/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um6_driver");

  // Load parameters from private node handle.
  std::string port;
  int32_t baud;

  ros::NodeHandle imu_nh("imu"), private_nh("~");
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int32_t>("baud", baud, 115200);

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

  sensor_msgs::Imu imu_msg;
  double linear_acceleration_stdev, angular_velocity_stdev;
  private_nh.param<std::string>("frame_id", imu_msg.header.frame_id, "imu_link");
  // Defaults obtained experimentally from hardware, no device spec exists
  private_nh.param<double>("linear_acceleration_stdev", linear_acceleration_stdev, 0.06);
  private_nh.param<double>("angular_velocity_stdev", angular_velocity_stdev, 0.005);

  double linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev;
  double angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev;

  // Enable converting from NED to ENU by default
  bool tf_ned_to_enu;
  private_nh.param<bool>("tf_ned_to_enu", tf_ned_to_enu, true);

  // Use MagneticField message rather than Vector3Stamped.
  bool use_magnetic_field_msg;
  private_nh.param<bool>("use_magnetic_field_msg", use_magnetic_field_msg, false);

  imu_msg.linear_acceleration_covariance[0] = linear_acceleration_cov;
  imu_msg.linear_acceleration_covariance[4] = linear_acceleration_cov;
  imu_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

  imu_msg.angular_velocity_covariance[0] = angular_velocity_cov;
  imu_msg.angular_velocity_covariance[4] = angular_velocity_cov;
  imu_msg.angular_velocity_covariance[8] = angular_velocity_cov;

  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      ser.open();
    }
    catch(const serial::IOException& e)
    {
      ROS_DEBUG("Unable to connect to port.");
    }
    if (ser.isOpen())
    {
      ROS_INFO("Successfully connected to serial port.");
      first_failure = true;
      try
      {
        um6::Comms sensor(&ser);
        configureSensor(&sensor, &private_nh);
        um6::Registers registers;
        ros::ServiceServer srv = imu_nh.advertiseService<um6::Reset::Request, um6::Reset::Response>(
                                   "reset", boost::bind(handleResetService, &sensor, _1, _2));

        while (ros::ok())
        {
          if (sensor.receive(&registers) == TRIGGER_PACKET)
          {
            // Triggered by arrival of final message in group.
            imu_msg.header.stamp = ros::Time::now();
            publishMsgs(registers, &imu_nh, imu_msg, tf_ned_to_enu, use_magnetic_field_msg);
            ros::spinOnce();
          }
        }
      }
      catch(const std::exception& e)
      {
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("Attempting reconnection after error.");
        ros::Duration(1.0).sleep();
      }
    }
    else
    {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device "
                           << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }
  }
}
