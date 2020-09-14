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
 * @file packet_storage.cpp
 * @brief Container for data coming in from the KVH
 * @author Trevor Bostic
 */

#include "kvh_geo_fog_3d_driver.hpp"
#include "kvh_geo_fog_3d_packet_storage.hpp"

namespace kvh
{

  KvhPacketStorage::KvhPacketStorage(){};

  /**
   * @fn KvhPacketStorage::Init
   * @brief Correctly sets up a KvhPacketMap for the requested packets
   * @param _packRequest [in] List of packets we'd like to request from the sensor
   * @return [int]:
   *    0 = Success,
   *    -1 = Failure, duplicate id found
   *    -2 = Failure, unsupported id found
   */
  int KvhPacketStorage::Init(KvhPacketRequest& _packRequest)
  {

    std::set<packet_id_e> packetIdList;
    for (int i = 0; i < _packRequest.size(); i++)
    {
      packet_id_e packEnum = _packRequest.at(i).first;

      // Check for duplicates
      if (packetIdList.count(packEnum) > 0)
      {
        return -1;
      }
      else
      {
        packetIdList.insert(packEnum);
      }
    

      /*
       * General form for below:
       *  case (packetId):
       *    packetMap_[packetId] = pair(false, shared_ptr(packet_struct_t))
       */
      switch (packEnum)
      {
        case packet_id_system_state:
          packetMap_[packet_id_system_state] = std::make_pair(false, std::make_shared<system_state_packet_t>());
          break;
        case packet_id_unix_time:
          packetMap_[packet_id_unix_time] = std::make_pair(false, std::make_shared<unix_time_packet_t>());
          break;
        case packet_id_raw_sensors:
          packetMap_[packet_id_raw_sensors] = std::make_pair(false, std::make_shared<raw_sensors_packet_t>());
          break;
        case packet_id_satellites:
          packetMap_[packet_id_satellites] = std::make_pair(false, std::make_shared<satellites_packet_t>());
          break;
        case packet_id_satellites_detailed:
          packetMap_[packet_id_satellites_detailed] = std::make_pair(false, std::make_shared<detailed_satellites_packet_t>());
          break;
        case packet_id_local_magnetics:
          packetMap_[packet_id_local_magnetics] = std::make_pair(false, std::make_shared<local_magnetics_packet_t>());
          break;
        case packet_id_utm_position:
          packetMap_[packet_id_utm_position] = std::make_pair(false, std::make_shared<utm_fix>());
          break;
        case packet_id_ecef_position:
          packetMap_[packet_id_ecef_position] = std::make_pair(false, std::make_shared<ecef_position_packet_t>());
          break;
        case packet_id_north_seeking_status:
          packetMap_[packet_id_north_seeking_status] = std::make_pair(false, std::make_shared<north_seeking_status_packet_t>());
          break;
        case packet_id_euler_orientation_standard_deviation:
          packetMap_[packet_id_euler_orientation_standard_deviation] = std::make_pair(false, std::make_shared<euler_orientation_standard_deviation_packet_t>());
          break;
        case packet_id_odometer_state:
          packetMap_[packet_id_odometer_state] = std::make_pair(false, std::make_shared<odometer_state_packet_t>());
          break;
        case packet_id_raw_gnss:
          packetMap_[packet_id_raw_gnss] = std::make_pair(false, std::make_shared<raw_gnss_packet_t>());
          break;
        default:
          return -2;
      }
    }

    // Will return 0 if we support all, or the number of entered id's we don't support if >0
    return 0;
  } // END CreatePacketMap()

  /**
   * @fn KvhPacketStorage::SetPacketUpdated
   * @param [in] _packetId The packet we are setting the updated status of
   * @param [in] _updateStatus The IsUpdated value for the packet
   * @return int
   *  0 = Success
   *  -1 = Packet id not in map
   */
  int KvhPacketStorage::SetPacketUpdated(packet_id_e _packetId, bool _updateStatus)
  {
    if(this->Contains(_packetId))
    {
      packetMap_[_packetId].first = _updateStatus;
      return 0;
    }

    return -1;
  }

  /**
   * @fn KvhPacketStorage::PacketIsUpdated
   * @param [in] _packetId The id associated with the packet we are checking for updates
   * @return bool True if the packet has updated, false if not updated, or packet not found in map
   */
  bool KvhPacketStorage::PacketIsUpdated(packet_id_e _packetId)
  {
    if (this->Contains(_packetId))
    {
      return packetMap_[_packetId].first;
    }
  
    return false;
  }

  /**
   * @fn KvhPacketStorage::Contains
   * @param [in] _packetId The packet id we are looking
   * @return bool True if in our map, false otherwise
   */
  bool KvhPacketStorage::Contains(packet_id_e _packetId)
  {
    return packetMap_.count(_packetId) > 0;
  }

  /**
   * @fn KvhPacketStorage::Size
   * @return The number of different packet types currently stored
   */
  int KvhPacketStorage::Size()
  {
    return packetMap_.size();
  }

  /**
   * @fn KvhPacketStorage::PrintPacketTypes
   * Prints the string types of all supported packets
   * \todo This maybe should be moved to global scope
   */
  void KvhPacketStorage::PrintPacketTypes()
  {
    for(auto it = packetTypeStr_.cbegin(); it != packetTypeStr_.cend(); it++)
    {
      printf("Packet Id: %d, Packet Type: %s\n", it->first, packetTypeStr_[it->first].c_str());
    }
  }

  /**
   * @fn KvhPacketStorage::PrintPacketSizes
   * Prints the sizes of all currently supported packets
   * \todo This maybe should be moved to global scope
   */
  void KvhPacketStorage::PrintPacketSizes()
  {
    for (auto it = packetSize_.cbegin(); it != packetSize_.cend(); it++)
    {
      printf("Packet id: %d, Size: %d\n", it->first, packetSize_[it->first]);
    }
  }
}
