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
 * @file kvh_geo_fog_3d_global_vars.hpp
 * @brief global variables used to store packet information.
 * @author Trevor Bostic
 */

#pragma once

#include <map>
#include <set>

// GEO-FOG
#include "an_packet_protocol.h"
#include "spatial_packets.h"

namespace kvh
{
  extern std::set<packet_id_e> supportedPackets_; ///< Set of packets containing all packet_id's we support
  extern std::map<packet_id_e, int> packetSize_; ///< Map relating packet id's to the associated struct size. Used for baudrate calculation
  extern std::map<packet_id_e, std::string> packetTypeStr_; ///< Holds the string value for the different types of structs.

  /**
   * @struct utm_fix
   * The kvh outputs the utm zone number, which their packet does not have.
   * We are inheriting from their packet and adding the field they left off.
   */
  struct utm_fix : utm_position_packet_t
  {
    uint8_t zone_num;
  };
}
