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
 * @file kvh_geo_fog_3d_device_configuration.hpp
 * @brief Helper functions for configuring the hardware.
 * @author Trevor Bostic
 */

#pragma once

// STD
#include <stdint.h>

// KVH Geo Fog
#include "an_packet_protocol.h"
#include "spatial_packets.h"

// Driver
#include "kvh_geo_fog_3d_global_vars.hpp"
#include "kvh_geo_fog_3d_packet_storage.hpp"

namespace kvh
{

  class KvhDeviceConfig
  {
  private:
  public:
    /**
     * @code
     * kvh::KvhPacketRequest packRequest = {
     *    // Fill in packets
     * }
     * packet_periods_packet_t periodsPacket;
     * if (kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(packRequest, periodsPacket) != 0)
     *    // Error
     * // Use packet periods packet
     * @endcode
     */
    static int CreatePacketPeriodsPacket(KvhPacketRequest &_packetsRequested, packet_periods_packet_t &_packetPeriods);

    /**
     * @code
     * filter_options_packet_t filterOptions;
     * if (kvh::KvhDeviceConfig::CreateFilterOptionsPacket(filterOptions, ... // Additional options) != 0)
     *    // Error
     * // Use filter options
     * @endcode
     */
    static int CreateFilterOptionsPacket(
      filter_options_packet_t &,
      bool _permanent = true,
      uint8_t _vehicle_type = vehicle_type_car,
      bool _internal_gnss_enabled = true,
      bool _atmospheric_altitude_enabled = true,
      bool _velocity_heading_enabled = true,
      bool _reversing_detection_enabled = true,
      bool _motion_analysis_enabled = true);

    /**
     * @code
     * std::string port{'/dev/USB0'};
     * int curBaud = 115200;
     * int prevBaud = 9600;
     * kvh::KvhDeviceConfig::SetBaudRate(port, curBaud, prevBaud);
     * @endcode
     */
    static int SetBaudRate(std::string _port, int _curBaudRate, int _primaryBaudRate, int _gpioBaudRate = 115200, int _auxBaudRate = 115200);

    /**
     * @code
     * std::string port{'/dev/ttyUSB1'};
     * int curBaud = FindCurrentBaudRate(port);
     * @endcode
     */
    static int FindCurrentBaudRate(std::string, int);

    /**
     * @code
     * kvh::KvhPacketRequest packetRequest = {
     *    // Create packet timer
     * }
     * kvh::KvhDeviceConfig::CalculateRequiredBaud(packetRequest);
     * @endcode
     */
    static int CalculateRequiredBaud(KvhPacketRequest &);
  };

} // namespace kvh
