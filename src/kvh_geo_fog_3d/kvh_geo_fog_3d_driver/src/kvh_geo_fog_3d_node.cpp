/*********************************************************************
 * Software License Agreement (Apache 2.0)
 * 
 *  Copyright (c) 2019, The MITRE Corporation.
 *  All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Sections of this project contains content developed by The MITRE Corporation.
 * If this code is used in a deployment or embedded within another project,
 * it is requested that you send an email to opensource@mitre.org in order to
 * let us know where this software is being used.
 *********************************************************************/

/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 *
 * @todo Switch publishers to DiagnosticPublisher, which will let us track frequencies (see http://docs.ros.org/api/diagnostic_updater/html/classdiagnostic__updater_1_1DiagnosedPublisher.html)
 */

// STD
#include "unistd.h"
#include <map>
#include <cmath>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "kvh_geo_fog_3d_global_vars.hpp"
#include "spatial_packets.h"
#include "kvh_diagnostics_container.hpp"

// ROS
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>

// Custom ROS msgs
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DSystemState.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DSatellites.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DDetailSatellites.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DLocalMagneticField.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DUTMPosition.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DECEFPos.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DNorthSeekingInitStatus.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DOdometerState.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DRawGNSS.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DRawSensors.h>

// Standard ROS msgs
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"

// Bounds on [-pi, pi)
inline double BoundFromNegPiToPi(const double &_value)
{
  double num = std::fmod(_value, (2*M_PI));
  if (num > M_PI)
  {
    num = num - (2*M_PI);
  }
  return num;
} //end: BoundFromNegPiToPi(double* _value)

inline double BoundFromNegPiToPi(const float &_value)
{
  double num = std::fmod(_value, (2*M_PI));
  if (num > M_PI)
  {
    num = num - (2*M_PI);
  }
  return num;
} //end: BoundFromNegPiToPi(const float& _value)

// Bounds on [-pi, pi)
inline double BoundFromZeroTo2Pi(const double &_value)
{
  return std::fmod(_value, (2 * M_PI));
} //end: BoundFromZeroTo2Pi(double* _value)

inline double BoundFromZeroTo2Pi(const float &_value)
{
  return std::fmod(_value, (2 * M_PI));
} //end: BoundFromZeroTo2Pi(const float& _value)

void SetupUpdater(diagnostic_updater::Updater *_diagnostics, mitre::KVH::DiagnosticsContainer *_diagContainer)
{
  _diagnostics->setHardwareID("KVH GEO FOG 3D"); ///< @todo This should probably contain the serial number of the unit, but we only get that after a message read
  /**
   * @todo Add a diagnostics expected packet frequency for important packets and verify
   */
  _diagnostics->add("KVH System", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateSystemStatus);
  _diagnostics->add("KVH Filters", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateFilterStatus);
}

int GetInitOptions(ros::NodeHandle& _node, kvh::KvhInitOptions& _initOptions)
{

  // Check if the port has been set on the ros param server
  _node.getParam("port", _initOptions.port);
  _node.getParam("baud", _initOptions.baudRate);
  _node.getParam("debug", _initOptions.debugOn);

  int filterVehicleType;
  if (_node.getParam("filterVehicleType", filterVehicleType))
  {
    // node.getParam doesn't have an overload for uint8_t
    _initOptions.filterVehicleType = filterVehicleType;
  }

  _node.getParam("atmosphericAltitudeEnabled", _initOptions.atmosphericAltitudeEnabled);
  _node.getParam("velocityHeadingEnabled", _initOptions.velocityHeadingEnabled);
  _node.getParam("reversingDetectionEnabled", _initOptions.reversingDetectionEnabled);
  _node.getParam("motionAnalysisEnabled", _initOptions.motionAnalysisEnabled);
  _node.getParam("odomPulseToMeters", _initOptions.odomPulseToMeters);
  
  ROS_INFO_STREAM("Port: " << _initOptions.port);
  ROS_INFO_STREAM("Baud: " << _initOptions.baudRate);
  ROS_INFO_STREAM("Debug: " << _initOptions.debugOn);
  ROS_INFO_STREAM("Filter Vehicle Type: " << (int) _initOptions.filterVehicleType);
  ROS_INFO_STREAM("Atmospheric Altitude Enabled: " << _initOptions.atmosphericAltitudeEnabled);
  ROS_INFO_STREAM("Velocity Heading Enabled: " << _initOptions.velocityHeadingEnabled);
  ROS_INFO_STREAM("Reversing Detection Enabled: " << _initOptions.reversingDetectionEnabled);
  ROS_INFO_STREAM("Motion Analysis Enabled: " << _initOptions.motionAnalysisEnabled);
  ROS_INFO_STREAM("Odometer Pulses to Meters: " << _initOptions.odomPulseToMeters);

  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kvh_geo_fog_3d_driver");

  ros::NodeHandle node("~");
  ros::Rate rate(50); // 50hz by default, may eventually make settable parameter

  diagnostic_updater::Updater diagnostics;
  mitre::KVH::DiagnosticsContainer diagContainer;
  SetupUpdater(&diagnostics, &diagContainer);

  // Custom msg publishers
  std::map<packet_id_e, ros::Publisher> kvhPubMap{
      {packet_id_system_state, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DSystemState>("kvh_system_state", 1)},
      {packet_id_satellites, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DSatellites>("kvh_satellites", 1)},
      {packet_id_satellites_detailed, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DDetailSatellites>("kvh_detailed_satellites", 1)},
      {packet_id_local_magnetics, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DLocalMagneticField>("kvh_local_magnetics", 1)},
      {packet_id_utm_position, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DUTMPosition>("kvh_utm_position", 1)},
      {packet_id_ecef_position, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DECEFPos>("kvh_ecef_pos", 1)},
      {packet_id_north_seeking_status, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DNorthSeekingInitStatus>("kvh_north_seeking_status", 1)},
      {packet_id_odometer_state, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DOdometerState>("kvh_odometer_state", 1)},
      {packet_id_raw_sensors, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DRawSensors>("kvh_raw_sensors", 1)},
      {packet_id_raw_gnss, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DRawGNSS>("kvh_raw_gnss", 1)}};

  // Publishers for standard ros messages
  ros::Publisher imuDataRawPub = node.advertise<sensor_msgs::Imu>("imu/data_raw_frd", 1);
  ros::Publisher imuDataRawFLUPub = node.advertise<sensor_msgs::Imu>("imu/data_raw_flu", 1);
  ros::Publisher imuDataNEDPub = node.advertise<sensor_msgs::Imu>("imu/data_ned", 1);
  ros::Publisher imuDataENUPub = node.advertise<sensor_msgs::Imu>("imu/data_enu", 1);
  ros::Publisher imuDataRpyNEDPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_ned", 1);
  ros::Publisher imuDataRpyNEDDegPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_ned_deg", 1);
  ros::Publisher imuDataRpyENUPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_enu", 1);
  ros::Publisher imuDataRpyENUDegPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_enu_deg", 1);
  ros::Publisher navSatFixPub = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
  ros::Publisher rawNavSatFixPub = node.advertise<sensor_msgs::NavSatFix>("gps/raw_fix", 1);
  ros::Publisher magFieldPub = node.advertise<sensor_msgs::MagneticField>("mag", 1);
  ros::Publisher odomPubNED = node.advertise<nav_msgs::Odometry>("gps/utm_ned", 1);
  ros::Publisher odomPubENU = node.advertise<nav_msgs::Odometry>("gps/utm_enu", 1);
  ros::Publisher odomStatePub = node.advertise<nav_msgs::Odometry>("odom/wheel_encoder", 1);
  ros::Publisher rawSensorImuPub = node.advertise<sensor_msgs::Imu>("imu/raw_sensor_frd", 1);
  ros::Publisher rawSensorImuFluPub = node.advertise<sensor_msgs::Imu>("imu/raw_sensor_flu", 1);

  //////////////////////////
  // KVH Setup
  //////////////////////////

  // To get packets from the driver, we first create a vector
  // that holds a pair containing the packet id and the desired frequency for it to be published
  // See documentation for all id's.
  typedef std::pair<packet_id_e, int> freqPair;

  kvh::KvhPacketRequest packetRequest{
      freqPair(packet_id_euler_orientation_standard_deviation, 50),
      freqPair(packet_id_system_state, 50),
      freqPair(packet_id_satellites, 10),
      freqPair(packet_id_satellites_detailed, 1),
      freqPair(packet_id_local_magnetics, 50),
      freqPair(packet_id_utm_position, 50),
      freqPair(packet_id_ecef_position, 50),
      freqPair(packet_id_north_seeking_status, 50),
      freqPair(packet_id_odometer_state, 50),
      freqPair(packet_id_raw_sensors, 50),
      freqPair(packet_id_raw_gnss, 50),
  };

  kvh::Driver kvhDriver;
  kvh::KvhInitOptions initOptions;

  if (GetInitOptions(node, initOptions) < 0)
  {
    ROS_ERROR("Unable to get init options. Exiting.");
    exit(1);
  }

  int errorCode;
  if ((errorCode = kvhDriver.Init(initOptions.port, packetRequest, initOptions)) < 0)
  {
    ROS_ERROR("Unable to initialize driver. Error Code %d", errorCode);
    exit(1);
  };

  // Declare these for reuse
  system_state_packet_t systemStatePacket;
  satellites_packet_t satellitesPacket;
  detailed_satellites_packet_t detailSatellitesPacket;
  local_magnetics_packet_t localMagPacket;
  kvh::utm_fix utmPosPacket;
  ecef_position_packet_t ecefPosPacket;
  north_seeking_status_packet_t northSeekingStatPacket;
  euler_orientation_standard_deviation_packet_t eulStdDevPack;
  odometer_state_packet_t odomStatePacket;
  raw_sensors_packet_t rawSensorsPacket;
  raw_gnss_packet_t rawGnssPacket;

  while (ros::ok())
  {
    // Collect packet data
    kvhDriver.Once();

    // Create header we will use for all messages. Important to have timestamp the same
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    ///////////////////////////////////////////
    // CUSTOM ROS MESSAGES
    ///////////////////////////////////////////

    // SYSTEM STATE PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_system_state))
    {
      ROS_DEBUG("System state packet has updated. Publishing...");
      
      kvhDriver.GetPacket(packet_id_system_state, systemStatePacket);

      kvh_geo_fog_3d_msgs::KvhGeoFog3DSystemState sysStateMsg;
      sysStateMsg.header = header;

      sysStateMsg.system_status = systemStatePacket.system_status.r;
      sysStateMsg.filter_status = systemStatePacket.filter_status.r;

      sysStateMsg.unix_time_s = systemStatePacket.unix_time_seconds;
      sysStateMsg.unix_time_us = systemStatePacket.microseconds;

      sysStateMsg.latitude_rad = systemStatePacket.latitude;
      sysStateMsg.longitude_rad = systemStatePacket.longitude;
      sysStateMsg.height_m = systemStatePacket.height;

      sysStateMsg.absolute_velocity_north_mps = systemStatePacket.velocity[0];
      sysStateMsg.absolute_velocity_east_mps = systemStatePacket.velocity[1];
      sysStateMsg.absolute_velocity_down_mps = systemStatePacket.velocity[2];

      sysStateMsg.body_acceleration_x_mps = systemStatePacket.body_acceleration[0];
      sysStateMsg.body_acceleration_y_mps = systemStatePacket.body_acceleration[1];
      sysStateMsg.body_acceleration_z_mps = systemStatePacket.body_acceleration[2];

      sysStateMsg.g_force_g = systemStatePacket.g_force;

      sysStateMsg.roll_rad = systemStatePacket.orientation[0];
      sysStateMsg.pitch_rad = systemStatePacket.orientation[1];
      sysStateMsg.heading_rad = systemStatePacket.orientation[2];

      sysStateMsg.angular_velocity_x_rad_per_s = systemStatePacket.angular_velocity[0];
      sysStateMsg.angular_velocity_y_rad_per_s = systemStatePacket.angular_velocity[1];
      sysStateMsg.angular_velocity_z_rad_per_s = systemStatePacket.angular_velocity[2];

      sysStateMsg.latitude_stddev_m = systemStatePacket.standard_deviation[0];
      sysStateMsg.longitude_stddev_m = systemStatePacket.standard_deviation[1];
      sysStateMsg.height_stddev_m = systemStatePacket.standard_deviation[2];

      //Update diagnostics container from this message
      diagContainer.SetSystemStatus(systemStatePacket.system_status.r);
      diagContainer.SetFilterStatus(systemStatePacket.filter_status.r);

      kvhPubMap[packet_id_system_state].publish(sysStateMsg);
    }

    // SATELLITES PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_satellites))
    {
      ROS_DEBUG("Satellites packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_satellites, satellitesPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DSatellites satellitesMsg;

      satellitesMsg.header = header;
      satellitesMsg.hdop = satellitesPacket.hdop;
      satellitesMsg.vdop = satellitesPacket.vdop;
      satellitesMsg.gps_satellites = satellitesPacket.gps_satellites;
      satellitesMsg.glonass_satellites = satellitesPacket.glonass_satellites;
      satellitesMsg.beidou_satellites = satellitesPacket.beidou_satellites;
      satellitesMsg.galileo_satellites = satellitesPacket.sbas_satellites;

      kvhPubMap[packet_id_satellites].publish(satellitesMsg);
    }

    // SATELLITES DETAILED
    if (kvhDriver.PacketIsUpdated(packet_id_satellites_detailed))
    {
      ROS_DEBUG("Detailed satellites packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_satellites_detailed, detailSatellitesPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DDetailSatellites detailSatellitesMsg;

      detailSatellitesMsg.header = header;

      // MAXIMUM_DETAILED_SATELLITES is defined as 32 in spatial_packets.h
      // We must check if each field equals 0 as that denotes the end of the array
      for (int i = 0; i < MAXIMUM_DETAILED_SATELLITES; i++)
      {
        satellite_t satellite = detailSatellitesPacket.satellites[i];

        // Check if all fields = 0, if so then we should end our loop
        if (satellite.satellite_system == 0 && satellite.number == 0 &&
            satellite.frequencies.r == 0 && satellite.elevation == 0 &&
            satellite.azimuth == 0 && satellite.snr == 0)
        {
          break;
        }

        detailSatellitesMsg.satellite_system.push_back(satellite.satellite_system);
        detailSatellitesMsg.satellite_number.push_back(satellite.number);
        detailSatellitesMsg.satellite_frequencies.push_back(satellite.frequencies.r);
        detailSatellitesMsg.elevation_deg.push_back(satellite.elevation);
        detailSatellitesMsg.azimuth_deg.push_back(satellite.azimuth);
        detailSatellitesMsg.snr_decibal.push_back(satellite.snr);
      }

      kvhPubMap[packet_id_satellites_detailed].publish(detailSatellitesMsg);
    }

    // LOCAL MAGNETICS PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
    {
      ROS_DEBUG("Local magnetics packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DLocalMagneticField localMagFieldMsg;

      localMagFieldMsg.header = header;
      localMagFieldMsg.loc_mag_field_x_mG = localMagPacket.magnetic_field[0];
      localMagFieldMsg.loc_mag_field_y_mG = localMagPacket.magnetic_field[1];
      localMagFieldMsg.loc_mag_field_z_mG = localMagPacket.magnetic_field[2];

      kvhPubMap[packet_id_local_magnetics].publish(localMagFieldMsg);
    }

    // UTM POSITION PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
    {
      ROS_DEBUG("UTM Position packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DUTMPosition utmPosMsg;

      utmPosMsg.header = header;
      utmPosMsg.northing_m = utmPosPacket.position[0];
      utmPosMsg.easting_m = utmPosPacket.position[1];
      utmPosMsg.height_m = utmPosPacket.position[2];
      utmPosMsg.zone_character = utmPosPacket.zone;

      kvhPubMap[packet_id_utm_position].publish(utmPosMsg);
    }

    // ECEF POSITION PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_ecef_position))
    {
      ROS_DEBUG("ECEF position packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_ecef_position, ecefPosPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DECEFPos ecefPosMsg;

      ecefPosMsg.header = header;
      ecefPosMsg.ecef_x_m = ecefPosPacket.position[0];
      ecefPosMsg.ecef_y_m = ecefPosPacket.position[1];
      ecefPosMsg.ecef_z_m = ecefPosPacket.position[2];

      kvhPubMap[packet_id_ecef_position].publish(ecefPosMsg);
    }

    // NORTH SEEKING STATUS PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_north_seeking_status))
    {
      ROS_DEBUG("North seeking status packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_north_seeking_status, northSeekingStatPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DNorthSeekingInitStatus northSeekInitStatMsg;

      northSeekInitStatMsg.header = header;

      northSeekInitStatMsg.flags = northSeekingStatPacket.north_seeking_status.r;
      northSeekInitStatMsg.quadrant_1_data_per = northSeekingStatPacket.quadrant_data_collection_progress[0];
      northSeekInitStatMsg.quadrant_2_data_per = northSeekingStatPacket.quadrant_data_collection_progress[1];
      northSeekInitStatMsg.quadrant_3_data_per = northSeekingStatPacket.quadrant_data_collection_progress[2];
      northSeekInitStatMsg.quadrant_4_data_per = northSeekingStatPacket.quadrant_data_collection_progress[3];

      northSeekInitStatMsg.current_rotation_angle_rad = northSeekingStatPacket.current_rotation_angle;

      northSeekInitStatMsg.current_gyro_bias_sol_x_rad_s = northSeekingStatPacket.current_gyroscope_bias_solution[0];
      northSeekInitStatMsg.current_gyro_bias_sol_y_rad_s = northSeekingStatPacket.current_gyroscope_bias_solution[1];
      northSeekInitStatMsg.current_gyro_bias_sol_z_rad_s = northSeekingStatPacket.current_gyroscope_bias_solution[2];
      northSeekInitStatMsg.current_gyro_bias_sol_error_per = northSeekingStatPacket.current_gyroscope_bias_solution_error;

      kvhPubMap[packet_id_north_seeking_status].publish(northSeekInitStatMsg);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
    {
      ROS_DEBUG("Odometer stage updated. Publishing...");
      kvhDriver.GetPacket(packet_id_odometer_state, odomStatePacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DOdometerState odometerStateMsg;

      odometerStateMsg.header = header;
      odometerStateMsg.odometer_pulse_count = odomStatePacket.pulse_count;
      odometerStateMsg.odometer_distance_m = odomStatePacket.distance;
      odometerStateMsg.odometer_speed_mps = odomStatePacket.speed;
      odometerStateMsg.odometer_slip_m = odomStatePacket.slip;
      odometerStateMsg.odometer_active = odomStatePacket.active;

      kvhPubMap[packet_id_odometer_state].publish(odometerStateMsg);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_sensors))
    {
      ROS_DEBUG("Raw sensors packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_raw_sensors, rawSensorsPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DRawSensors rawSensorMsg;

      rawSensorMsg.header = header;

      rawSensorMsg.accelerometer_x_mpss = rawSensorsPacket.accelerometers[0];
      rawSensorMsg.accelerometer_y_mpss = rawSensorsPacket.accelerometers[1];
      rawSensorMsg.accelerometer_z_mpss = rawSensorsPacket.accelerometers[2];

      rawSensorMsg.gyro_x_rps = rawSensorsPacket.gyroscopes[0];
      rawSensorMsg.gyro_y_rps = rawSensorsPacket.gyroscopes[1];
      rawSensorMsg.gyro_z_rps = rawSensorsPacket.gyroscopes[2];

      rawSensorMsg.magnetometer_x_mG = rawSensorsPacket.magnetometers[0];
      rawSensorMsg.magnetometer_y_mG = rawSensorsPacket.magnetometers[1];
      rawSensorMsg.magnetometer_z_mG = rawSensorsPacket.magnetometers[2];

      rawSensorMsg.imu_temp_c = rawSensorsPacket.imu_temperature;
      rawSensorMsg.pressure_pa = rawSensorsPacket.pressure;
      rawSensorMsg.pressure_temp_c = rawSensorsPacket.pressure_temperature;

      kvhPubMap[packet_id_raw_sensors].publish(rawSensorMsg);
    }

    /**
     * @attention The raw gnss has an additional field called floating
     * ambiguity heading. It is not implemented in their api. If we wish to
     * implement this, we would need to make an extension similar to what we
     * did for the UTM packet.
     */
    if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
    {
      ROS_DEBUG("Raw GNSS packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_raw_gnss, rawGnssPacket);
      kvh_geo_fog_3d_msgs::KvhGeoFog3DRawGNSS rawGnssMsg;

      rawGnssMsg.header = header;
      rawGnssMsg.unix_time_s = rawGnssPacket.unix_time_seconds;
      rawGnssMsg.unix_time_us = rawGnssPacket.microseconds;

      rawGnssMsg.latitude_rad = rawGnssPacket.position[0];
      rawGnssMsg.longitude_rad = rawGnssPacket.position[1];
      rawGnssMsg.height_m = rawGnssPacket.position[2];

      rawGnssMsg.latitude_stddev_m = rawGnssPacket.position_standard_deviation[0];
      rawGnssMsg.longitude_stddev_m = rawGnssPacket.position_standard_deviation[1];
      rawGnssMsg.height_stddev_m = rawGnssPacket.position_standard_deviation[2];

      rawGnssMsg.vel_north_m = rawGnssPacket.velocity[0];
      rawGnssMsg.vel_east_m = rawGnssPacket.velocity[1];
      rawGnssMsg.vel_down_m = rawGnssPacket.velocity[2];

      rawGnssMsg.tilt_rad = rawGnssPacket.tilt;
      rawGnssMsg.tilt_stddev_rad = rawGnssPacket.tilt_standard_deviation;
      
      rawGnssMsg.heading_rad = rawGnssPacket.heading;
      rawGnssMsg.heading_stddev_rad = rawGnssPacket.heading_standard_deviation;

      rawGnssMsg.gnss_fix = rawGnssPacket.flags.b.fix_type;
      rawGnssMsg.doppler_velocity_valid = rawGnssPacket.flags.b.velocity_valid;
      rawGnssMsg.time_valid = rawGnssPacket.flags.b.time_valid;
      rawGnssMsg.external_gnss = rawGnssPacket.flags.b.external_gnss;
      rawGnssMsg.tilt_valid = rawGnssPacket.flags.b.tilt_valid;
      rawGnssMsg.heading_valid = rawGnssPacket.flags.b.heading_valid;
      
      kvhPubMap[packet_id_raw_gnss].publish(rawGnssMsg);
    }

    ////////////////////////////////////
    // STANDARD ROS MESSAGES
    ////////////////////////////////////

    if (kvhDriver.PacketIsUpdated(packet_id_system_state) && kvhDriver.PacketIsUpdated(packet_id_euler_orientation_standard_deviation))
    {
      kvhDriver.GetPacket(packet_id_system_state, systemStatePacket);
      kvhDriver.GetPacket(packet_id_euler_orientation_standard_deviation, eulStdDevPack);

      // IMU Message Structure: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
      // Header
      // Quaternion orientation
      // float64[9] orientation_covariance
      // Vector3 angular_velocity
      // float64[9] angular_velocity_covariance
      // Vector3 linear_acceleration
      // float64[9] linear_acceleration_covariance

      // [-pi,pi) bounded yaw
      double boundedBearingPiToPi = BoundFromNegPiToPi(systemStatePacket.orientation[2]);
      double boundedBearingZero2Pi = BoundFromZeroTo2Pi(systemStatePacket.orientation[2]);

      // ORIENTATION
      // All orientations from this sensor are w.r.t. true north.
      if (std::isnan(eulStdDevPack.standard_deviation[0]))
      {
        eulStdDevPack.standard_deviation[0] = 0;
        ROS_INFO("NAN Found");
      }

      if (std::isnan(eulStdDevPack.standard_deviation[1]))
      {
        eulStdDevPack.standard_deviation[1] = 0;
        ROS_INFO("NAN Found");
      }

      if (std::isnan(eulStdDevPack.standard_deviation[2]))
      {
        eulStdDevPack.standard_deviation[2] = 0;
        ROS_INFO("NAN Found");
      }

      tf2::Quaternion orientQuatNED;
      orientQuatNED.setRPY(
          systemStatePacket.orientation[0],
          systemStatePacket.orientation[1],
          boundedBearingZero2Pi);
      double orientCovNED[3] = {
          pow(eulStdDevPack.standard_deviation[0], 2),
          pow(eulStdDevPack.standard_deviation[1], 2),
          pow(eulStdDevPack.standard_deviation[2], 2)};

      tf2::Quaternion orientQuatENU;
      //For NED -> ENU transformation:
      //(X -> Y, Y -> -X, Z -> -Z, Yaw = -Yaw + 90 deg, Pitch -> Roll, and Roll -> Pitch)
      double unfixedEnuBearing = (-1 * boundedBearingZero2Pi) + (M_PI / 2.0);
      double enuBearing = BoundFromZeroTo2Pi(unfixedEnuBearing);
      orientQuatENU.setRPY(
          systemStatePacket.orientation[1], // ENU roll = NED pitch
          systemStatePacket.orientation[0], // ENU pitch = NED roll
          enuBearing                        // ENU bearing = -(NED bearing) + 90 degrees
      );
      double orientCovENU[3] = {
          pow(eulStdDevPack.standard_deviation[1], 2),
          pow(eulStdDevPack.standard_deviation[0], 2),
          pow(eulStdDevPack.standard_deviation[2], 2),
      };

      // DATA_RAW Topic
      sensor_msgs::Imu imuDataRaw;
      imuDataRaw.header = header;
      imuDataRaw.header.frame_id = "imu_link_frd";

      // ANGULAR VELOCITY
      imuDataRaw.angular_velocity.x = systemStatePacket.angular_velocity[0];
      imuDataRaw.angular_velocity.y = systemStatePacket.angular_velocity[1];
      imuDataRaw.angular_velocity.z = systemStatePacket.angular_velocity[2];
      // Leave covariance at 0 since we don't have it
      // imuDataRaw.angular_velocity_covariance[0]
      // imuDataRaw.angular_velocity_covariance[4]
      // imuDataRaw.angular_velocity_covariance[8]

      // LINEAR ACCELERATION
      imuDataRaw.linear_acceleration.x = systemStatePacket.body_acceleration[0];
      imuDataRaw.linear_acceleration.y = systemStatePacket.body_acceleration[1];
      imuDataRaw.linear_acceleration.z = systemStatePacket.body_acceleration[2];
      // Leave covariance at 0 since we don't have it
      // imuDataRaw.linear_acceleration_covariance[0]
      // imuDataRaw.linear_acceleration_covariance[4]
      // imuDataRaw.linear_acceleration_covariance[8]

      imuDataRawPub.publish(imuDataRaw);

      // DATA_RAW_FLU
      sensor_msgs::Imu imuDataRawFLU;
      imuDataRawFLU.header = header;
      imuDataRawFLU.header.frame_id = "imu_link_flu";

      // ANGULAR VELOCITY
      imuDataRawFLU.angular_velocity.x = systemStatePacket.angular_velocity[0];
      imuDataRawFLU.angular_velocity.y = -1 * systemStatePacket.angular_velocity[1];
      imuDataRawFLU.angular_velocity.z = -1 * systemStatePacket.angular_velocity[2]; // To account for east north up system
      // Leave covariance at 0 since we don't have it
      // imuDataRawFLU.angular_velocity_covariance[0]
      // imuDataRawFLU.angular_velocity_covariance[4]
      // imuDataRawFLU.angular_velocity_covariance[8]

      // LINEAR ACCELERATION
      imuDataRawFLU.linear_acceleration.x = systemStatePacket.body_acceleration[0];
      imuDataRawFLU.linear_acceleration.y = -1 * systemStatePacket.body_acceleration[1];
      imuDataRawFLU.linear_acceleration.z = -1 * systemStatePacket.body_acceleration[2];
      // Leave covariance at 0 since we don't have it
      // imuDataRawFLU.linear_acceleration_covariance[0]
      // imuDataRawFLU.linear_acceleration_covariance[4]
      // imuDataRawFLU.linear_acceleration_covariance[8]

      imuDataRawFLUPub.publish(imuDataRawFLU);

      ///////////////////////////////////////////////////////////////////////////////////////////////
      // DATA_NED topic
      ///////////////////////////////////////////////////////////////////////////////////////////////

      sensor_msgs::Imu imuDataNED;
      geometry_msgs::Vector3Stamped imuDataRpyNED;
      geometry_msgs::Vector3Stamped imuDataRpyNEDDeg;
      imuDataNED.header = header;
      imuDataNED.header.frame_id = "imu_link_frd";

      imuDataNED.orientation.x = orientQuatNED.x();
      imuDataNED.orientation.y = orientQuatNED.y();
      imuDataNED.orientation.z = orientQuatNED.z();
      imuDataNED.orientation.w = orientQuatNED.w();

      imuDataNED.orientation_covariance[0] = orientCovNED[0];
      imuDataNED.orientation_covariance[4] = orientCovNED[1];
      imuDataNED.orientation_covariance[8] = orientCovNED[2];

      imuDataRpyNED.header = header;
      imuDataRpyNED.header.frame_id = "imu_link_frd";
      imuDataRpyNED.vector.x = systemStatePacket.orientation[0];
      imuDataRpyNED.vector.y = systemStatePacket.orientation[1];
      imuDataRpyNED.vector.z = boundedBearingZero2Pi;

      imuDataRpyNEDDeg.header = header;
      imuDataRpyNEDDeg.header.frame_id = "imu_link_frd";
      imuDataRpyNEDDeg.vector.x = ((imuDataRpyNED.vector.x * 180.0) / M_PI);
      imuDataRpyNEDDeg.vector.y = ((imuDataRpyNED.vector.y * 180.0) / M_PI);
      imuDataRpyNEDDeg.vector.z = ((imuDataRpyNED.vector.z * 180.0) / M_PI);

      // ANGULAR VELOCITY
      imuDataNED.angular_velocity.x = systemStatePacket.angular_velocity[0];
      imuDataNED.angular_velocity.y = systemStatePacket.angular_velocity[1];
      imuDataNED.angular_velocity.z = systemStatePacket.angular_velocity[2];

      // LINEAR ACCELERATION
      imuDataNED.linear_acceleration.x = systemStatePacket.body_acceleration[0];
      imuDataNED.linear_acceleration.y = systemStatePacket.body_acceleration[1];
      imuDataNED.linear_acceleration.z = systemStatePacket.body_acceleration[2];

      imuDataNEDPub.publish(imuDataNED);
      imuDataRpyNEDPub.publish(imuDataRpyNED);
      imuDataRpyNEDDegPub.publish(imuDataRpyNEDDeg);

      ///////////////////////////////////////////////////////////////////////////////////////////////
      // DATA_ENU topic
      ///////////////////////////////////////////////////////////////////////////////////////////////

      sensor_msgs::Imu imuDataENU;
      geometry_msgs::Vector3Stamped imuDataRpyENU;
      geometry_msgs::Vector3Stamped imuDataRpyENUDeg;

      imuDataENU.header = header;
      imuDataENU.header.frame_id = "imu_link_flu";

      imuDataRpyENU.header = header;
      imuDataRpyENU.header.frame_id = "imu_link_flu";

      imuDataRpyENUDeg.header = header;
      imuDataRpyENUDeg.header.frame_id = "imu_link_flu";

      // ORIENTATION
      //Keep in mind that these are w.r.t. frame_id
      imuDataENU.orientation.x = orientQuatENU.x();
      imuDataENU.orientation.y = orientQuatENU.y();
      imuDataENU.orientation.z = orientQuatENU.z();
      imuDataENU.orientation.w = orientQuatENU.w();

      imuDataENU.orientation_covariance[0] = orientCovENU[1];
      imuDataENU.orientation_covariance[4] = orientCovENU[0];
      imuDataENU.orientation_covariance[8] = orientCovENU[2];

      imuDataRpyENU.vector.x = systemStatePacket.orientation[1];
      imuDataRpyENU.vector.y = systemStatePacket.orientation[0];
      imuDataRpyENU.vector.z = enuBearing;

      imuDataRpyENUDeg.vector.x = ((imuDataRpyENU.vector.x * 180.0) / M_PI);
      imuDataRpyENUDeg.vector.y = ((imuDataRpyENU.vector.y * 180.0) / M_PI);
      imuDataRpyENUDeg.vector.z = ((imuDataRpyENU.vector.z * 180.0) / M_PI);

      // ANGULAR VELOCITY
      // Keep in mind that for the sensor_msgs/Imu message, accelerations are
      // w.r.t the frame_id, which in this case is imu_link_flu.
      imuDataENU.angular_velocity.x = systemStatePacket.angular_velocity[0];
      imuDataENU.angular_velocity.y = -1 * systemStatePacket.angular_velocity[1];
      imuDataENU.angular_velocity.z = -1 * systemStatePacket.angular_velocity[2];

      // LINEAR ACCELERATION
      // Keep in mind that for the sensor_msgs/Imu message, accelerations are
      // w.r.t the frame_id, which in this case is imu_link_flu.
      imuDataENU.linear_acceleration.x = systemStatePacket.body_acceleration[0];
      imuDataENU.linear_acceleration.y = -1 * systemStatePacket.body_acceleration[1];
      imuDataENU.linear_acceleration.z = -1 * systemStatePacket.body_acceleration[2];

      // Publish
      imuDataENUPub.publish(imuDataENU);
      imuDataRpyENUPub.publish(imuDataRpyENU);
      imuDataRpyENUDegPub.publish(imuDataRpyENUDeg);

      // NAVSATFIX Message Structure: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html
      // NavSatStatus status
      // float64 latitude
      // float64 longitude
      // float64 altitude
      // float64[9] position_covariance // Uses East North Up (ENU) in row major order
      // uint8 position_covariance type
      sensor_msgs::NavSatFix navSatFixMsg;
      navSatFixMsg.header = header;
      navSatFixMsg.header.frame_id = "gps";

      // Set nav sat status
      int status = systemStatePacket.filter_status.b.gnss_fix_type;
      switch (status)
      {
      case 0:
        navSatFixMsg.status.status = navSatFixMsg.status.STATUS_NO_FIX;
        break;
      case 1:
      case 2:
        navSatFixMsg.status.status = navSatFixMsg.status.STATUS_FIX;
        break;
      case 3:
        navSatFixMsg.status.status = navSatFixMsg.status.STATUS_SBAS_FIX;
        break;
      default:
        navSatFixMsg.status.status = navSatFixMsg.status.STATUS_GBAS_FIX;
      }

      //NavSatFix specifies degrees as lat/lon, but KVH publishes in radians
      double latitude_deg = (systemStatePacket.latitude * 180.0) / M_PI;
      double longitude_deg = (systemStatePacket.longitude * 180.0) / M_PI;
      navSatFixMsg.latitude = latitude_deg;
      navSatFixMsg.longitude = longitude_deg;
      navSatFixMsg.altitude = systemStatePacket.height;
      navSatFixMsg.position_covariance_type = navSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
      // They use ENU for mat for this matrix. To me it makes sense that we should use
      // the longitude standard deviation for east.
      navSatFixMsg.position_covariance[0] = pow(systemStatePacket.standard_deviation[1], 2);
      navSatFixMsg.position_covariance[4] = pow(systemStatePacket.standard_deviation[0], 2);
      navSatFixMsg.position_covariance[8] = pow(systemStatePacket.standard_deviation[2], 2);

      navSatFixPub.publish(navSatFixMsg);

      // If we have system state and utm position we can publish odometry
      if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
      {
        // Odometry Message Structure: http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
        // Header
        // String child_frame_id \todo Fill out child frame id
        // PoseWithCovariance pose
        // --> Pose
        // -->-->Point position
        // -->-->Quaternion orientation
        // -->float6[36] covariance // 6x6 order is [x, y, z, X axis rot, y axis rot, z axis rot]
        // TwistWithCovariance twist
        // --> Twist // Velocity in free space
        // -->-->Vector3 linear
        // -->-->Vector3 angular
        // -->float64[36] covariance // 6x6 order is [x, y, z, x axis rot, y axis rot, z axis rot]

        // Since UTM is by default NED, we will publish a message like that and a message using
        // the ros ENU standard
        nav_msgs::Odometry odomMsgENU;
        nav_msgs::Odometry odomMsgNED;
        int error = kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);
        if (error < 0)
        {
          printf("Error Code: %d\n", error);
          printf("UTM PACKET: %f, %f, %f\n", utmPosPacket.position[0], utmPosPacket.position[1], utmPosPacket.position[2]);
        }

        odomMsgENU.header = header;
        odomMsgENU.header.frame_id = "utm_enu";     //The nav_msgs/Odometry "Pose" section should be in this frame
        odomMsgENU.child_frame_id = "imu_link_flu"; //The nav_msgs/Odometry "Twist" section should be in this frame

        odomMsgNED.header = header;
        odomMsgNED.header.frame_id = "utm_ned";     //The nav_msgs/Odometry "Pose" section should be in this frame
        odomMsgNED.child_frame_id = "imu_link_frd"; //The nav_msgs/Odometry "Twist" section should be in this frame

        // POSE
        // Position ENU
        odomMsgENU.pose.pose.position.x = utmPosPacket.position[1];
        odomMsgENU.pose.pose.position.y = utmPosPacket.position[0];
        odomMsgENU.pose.pose.position.z = -1 * utmPosPacket.position[2];
        // odomMsg.pose.covariance[0] =
        // odomMsg.pose.covariance[7] =
        // odomMsg.pose.covariance[14] =

        // Position NED
        odomMsgNED.pose.pose.position.x = utmPosPacket.position[0];
        odomMsgNED.pose.pose.position.y = utmPosPacket.position[1];
        odomMsgNED.pose.pose.position.z = utmPosPacket.position[2];
        // odomMsg.pose.covariance[0] =
        // odomMsg.pose.covariance[7] =
        // odomMsg.pose.covariance[14] =

        // Orientation ENU
        // Use orientation quaternion we created earlier
        odomMsgENU.pose.pose.orientation.x = orientQuatENU.x();
        odomMsgENU.pose.pose.orientation.y = orientQuatENU.y();
        odomMsgENU.pose.pose.orientation.z = orientQuatENU.z();
        odomMsgENU.pose.pose.orientation.w = orientQuatENU.w();
        // Use covariance array created earlier to fill out orientation covariance
        odomMsgENU.pose.covariance[21] = orientCovENU[0];
        odomMsgENU.pose.covariance[28] = orientCovENU[1];
        odomMsgENU.pose.covariance[35] = orientCovENU[2];

        // Orientation NED
        odomMsgNED.pose.pose.orientation.x = orientQuatNED.x();
        odomMsgNED.pose.pose.orientation.y = orientQuatNED.y();
        odomMsgNED.pose.pose.orientation.z = orientQuatNED.z();
        odomMsgNED.pose.pose.orientation.w = orientQuatNED.w();
        // Use covariance array created earlier to fill out orientation covariance
        odomMsgNED.pose.covariance[21] = orientCovNED[0];
        odomMsgNED.pose.covariance[28] = orientCovNED[1];
        odomMsgNED.pose.covariance[35] = orientCovNED[2];

        // TWIST
        // ENU uses FLU rates/accels
        odomMsgENU.twist.twist.linear.x = systemStatePacket.velocity[0];
        odomMsgENU.twist.twist.linear.y = (-1 * systemStatePacket.velocity[1]);
        odomMsgENU.twist.twist.linear.z = (-1 * systemStatePacket.velocity[2]);

        odomMsgENU.twist.twist.angular.x = systemStatePacket.angular_velocity[0];
        odomMsgENU.twist.twist.angular.y = (-1 * systemStatePacket.angular_velocity[1]);
        odomMsgENU.twist.twist.angular.z = (-1 * systemStatePacket.angular_velocity[2]);

        // NED uses FRD rates/accels
        odomMsgNED.twist.twist.linear.x = systemStatePacket.velocity[0];
        odomMsgNED.twist.twist.linear.y = systemStatePacket.velocity[1];
        odomMsgNED.twist.twist.linear.z = systemStatePacket.velocity[2];

        odomMsgNED.twist.twist.angular.x = systemStatePacket.angular_velocity[0];
        odomMsgNED.twist.twist.angular.y = systemStatePacket.angular_velocity[1];
        odomMsgNED.twist.twist.angular.z = systemStatePacket.angular_velocity[2];

        odomPubENU.publish(odomMsgENU);
        odomPubNED.publish(odomMsgNED);
      }
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
    {
      kvhDriver.GetPacket(packet_id_odometer_state, odomStatePacket);
      nav_msgs::Odometry kvhOdomStateMsg;

      kvhOdomStateMsg.header = header;
      //Technically this should be w.r.t the fixed frame locked to your wheel
      //with the encoder mounted. But, since I don't know what you're going to
      //call it, we'll stick with base_link.
      kvhOdomStateMsg.header.frame_id = "base_link";

      kvhOdomStateMsg.pose.pose.position.x = (odomStatePacket.pulse_count * initOptions.odomPulseToMeters);
      kvhOdomStateMsg.pose.pose.position.y = 0;
      kvhOdomStateMsg.pose.pose.position.z = 0;
      kvhOdomStateMsg.twist.twist.linear.x = odomStatePacket.speed;
      kvhOdomStateMsg.twist.twist.linear.y = 0;
      kvhOdomStateMsg.twist.twist.linear.z = 0;

      odomStatePub.publish(kvhOdomStateMsg);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
    {
      kvhDriver.GetPacket(packet_id_raw_gnss, rawGnssPacket);
      sensor_msgs::NavSatFix rawNavSatFixMsg;

      rawNavSatFixMsg.header = header;
      rawNavSatFixMsg.header.frame_id = "gps";

      //NavSatFix specifies degrees as lat/lon, but KVH publishes in radians
      double rawGnssLatitude_deg = (rawGnssPacket.position[0] * 180.0) / M_PI;
      double rawGnssLongitude_deg = (rawGnssPacket.position[1] * 180.0) / M_PI;
      rawNavSatFixMsg.latitude = rawGnssLatitude_deg;
      rawNavSatFixMsg.longitude = rawGnssLongitude_deg;
      rawNavSatFixMsg.altitude = rawGnssPacket.position[2];

      int status = systemStatePacket.filter_status.b.gnss_fix_type;
      switch (status)
      {
      case 0:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_NO_FIX;
        break;
      case 1:
      case 2:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_FIX;
        break;
      case 3:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_SBAS_FIX;
        break;
      default:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_GBAS_FIX;
      }

      rawNavSatFixMsg.position_covariance_type = rawNavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
      // They use ENU for mat for this matrix. To me it makes sense that we should use
      // the longitude standard deviation for east.
      rawNavSatFixMsg.position_covariance[0] = pow(rawGnssPacket.position_standard_deviation[1], 2);
      rawNavSatFixMsg.position_covariance[4] = pow(rawGnssPacket.position_standard_deviation[0], 2);
      rawNavSatFixMsg.position_covariance[8] = pow(rawGnssPacket.position_standard_deviation[2], 2);

      rawNavSatFixPub.publish(rawNavSatFixMsg);

    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_sensors))
    {
      kvhDriver.GetPacket(packet_id_raw_sensors, rawSensorsPacket);

      // DATA_RAW Topic
      sensor_msgs::Imu imuDataRaw;
      imuDataRaw.header = header;
      imuDataRaw.header.frame_id = "imu_link_frd";

      // ANGULAR VELOCITY
      imuDataRaw.angular_velocity.x = rawSensorsPacket.gyroscopes[0];
      imuDataRaw.angular_velocity.y = rawSensorsPacket.gyroscopes[1];
      imuDataRaw.angular_velocity.z = rawSensorsPacket.gyroscopes[2];

      // LINEAR ACCELERATION
      imuDataRaw.linear_acceleration.x = rawSensorsPacket.accelerometers[0];
      imuDataRaw.linear_acceleration.y = rawSensorsPacket.accelerometers[1];
      imuDataRaw.linear_acceleration.z = rawSensorsPacket.accelerometers[2];

      rawSensorImuPub.publish(imuDataRaw);

      // DATA_RAW_FLU
      sensor_msgs::Imu imuDataRawFLU;
      imuDataRawFLU.header = header;
      imuDataRawFLU.header.frame_id = "imu_link_flu";

      // ANGULAR VELOCITY
      imuDataRawFLU.angular_velocity.x = rawSensorsPacket.gyroscopes[0];
      imuDataRawFLU.angular_velocity.y = -1 * rawSensorsPacket.gyroscopes[1];
      imuDataRawFLU.angular_velocity.z = -1 * rawSensorsPacket.gyroscopes[2]; // To account for east north up system

      // LINEAR ACCELERATION
      imuDataRawFLU.linear_acceleration.x = rawSensorsPacket.accelerometers[0];
      imuDataRawFLU.linear_acceleration.y = -1 * rawSensorsPacket.accelerometers[1];
      imuDataRawFLU.linear_acceleration.z = -1 * rawSensorsPacket.accelerometers[2];

      rawSensorImuFluPub.publish(imuDataRawFLU);

      sensor_msgs::MagneticField magFieldMsg;

      magFieldMsg.header = header;
      magFieldMsg.header.frame_id = "imu_link_frd";
      magFieldMsg.magnetic_field.x = rawSensorsPacket.magnetometers[0];
      magFieldMsg.magnetic_field.y = rawSensorsPacket.magnetometers[1];
      magFieldMsg.magnetic_field.z = rawSensorsPacket.magnetometers[2];

      magFieldPub.publish(magFieldMsg);
      
    }

    // Set that we have read the latest versions of all packets. There is a small possibility we miss one packet
    // between using it above and setting it here.
    kvhDriver.SetPacketUpdated(packet_id_system_state, false);
    kvhDriver.SetPacketUpdated(packet_id_satellites, false);
    kvhDriver.SetPacketUpdated(packet_id_satellites_detailed, false);
    kvhDriver.SetPacketUpdated(packet_id_utm_position, false);
    kvhDriver.SetPacketUpdated(packet_id_ecef_position, false);
    kvhDriver.SetPacketUpdated(packet_id_north_seeking_status, false);
    kvhDriver.SetPacketUpdated(packet_id_local_magnetics, false);
    kvhDriver.SetPacketUpdated(packet_id_euler_orientation_standard_deviation, false);
    kvhDriver.SetPacketUpdated(packet_id_odometer_state, false);
    kvhDriver.SetPacketUpdated(packet_id_raw_gnss, false);
    kvhDriver.SetPacketUpdated(packet_id_raw_sensors, false);

    diagnostics.update();

    ros::spinOnce();
    rate.sleep();
    ROS_DEBUG("----------------------------------------");
  }

  diagnostics.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "Shutting down the KVH driver");
  kvhDriver.Cleanup();
}
