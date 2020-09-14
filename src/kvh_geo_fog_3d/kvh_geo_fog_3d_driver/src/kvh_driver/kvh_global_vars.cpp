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
 * @file kvh_global_vars.cpp
 * @brief KVH Geo Fog 3D driver global variables. Holds general packet data that is used throughout the driver.
 * @author Trevor Bostic
 */

#include "kvh_geo_fog_3d_global_vars.hpp"
#include <typeinfo>

namespace kvh
{
    // Keep a list of supported packets using KVH's packet id enum
    std::set<packet_id_e> supportedPackets_ = {
    packet_id_system_state,
    packet_id_unix_time,
    packet_id_raw_sensors,
    packet_id_satellites,
    packet_id_satellites_detailed,
    packet_id_local_magnetics,
    packet_id_utm_position,
    packet_id_ecef_position,
    packet_id_north_seeking_status,
    packet_id_euler_orientation_standard_deviation,
    packet_id_odometer_state,
    packet_id_raw_gnss,
};

// Keep lookup table of packet sizes. Used for calculating required baud rates
std::map<packet_id_e, int> packetSize_ = {
    {packet_id_system_state, sizeof(system_state_packet_t)},
    {packet_id_unix_time, sizeof(unix_time_packet_t)},
    {packet_id_raw_sensors, sizeof(raw_sensors_packet_t)},
    {packet_id_satellites, sizeof(satellites_packet_t)},
    {packet_id_satellites_detailed, sizeof(detailed_satellites_packet_t)},
    {packet_id_local_magnetics, sizeof(local_magnetics_packet_t)},
    {packet_id_utm_position, sizeof(utm_fix)},
    {packet_id_ecef_position, sizeof(ecef_position_packet_t)},
    {packet_id_north_seeking_status, sizeof(north_seeking_status_packet_t)},
    {packet_id_euler_orientation_standard_deviation, sizeof(euler_orientation_standard_deviation_packet_t)},
    {packet_id_odometer_state, sizeof(odometer_state_packet_t)},
    {packet_id_raw_gnss, sizeof(raw_gnss_packet_t)},
};

// String names of struct types. Used for assurance we have received the correct packet
std::map<packet_id_e, std::string> packetTypeStr_ = {
    {packet_id_system_state, typeid(system_state_packet_t).name()},
    {packet_id_unix_time, typeid(unix_time_packet_t).name()},
    {packet_id_raw_sensors, typeid(raw_sensors_packet_t).name()},
    {packet_id_satellites, typeid(satellites_packet_t).name()},
    {packet_id_satellites_detailed, typeid(detailed_satellites_packet_t).name()},
    {packet_id_local_magnetics, typeid(local_magnetics_packet_t).name()},
    {packet_id_utm_position, typeid(utm_fix).name()},
    {packet_id_ecef_position, typeid(ecef_position_packet_t).name()},
    {packet_id_north_seeking_status, typeid(north_seeking_status_packet_t).name()},
    {packet_id_euler_orientation_standard_deviation, typeid(euler_orientation_standard_deviation_packet_t).name()},
    {packet_id_odometer_state, typeid(odometer_state_packet_t).name()},
    {packet_id_raw_gnss, typeid(raw_gnss_packet_t).name()},
};
}
