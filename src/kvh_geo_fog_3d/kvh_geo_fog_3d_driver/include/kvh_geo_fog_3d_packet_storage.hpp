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
 * @file kvh_geo_fog_3d_packet_storage.hpp
 * @brief KVH Packet storing class header
 * 
 * This header file is for the class used to store packets. This
 * is not as simple a task as it seems due to the large number of
 * packet types and their respective functions.
 */

#pragma once

// STD
#include <map>
#include <memory> // share_ptr
#include <vector>
#include <typeinfo>


// GEO-FOG SDK
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "kvh_geo_fog_3d_global_vars.hpp"

namespace kvh
{

  /**
   * @typedef KvhPacketMap
   * Defines mapping from packet id's given in spatial_packets.h to a pair
   * that contains a pointer to a struct of the type represented by the id, and a boolean
   * denoting if the struct has been changed (for use when the map is passed to the
   * Driver::Once function).
   */
  typedef std::map<packet_id_e, std::pair<bool, std::shared_ptr<void>>> KvhPacketMap;

  /**
   * @typedef KvhPacketRequest
   * Defines the format for a packet, frequency (Hz) pair that should be passed in
   * by the user to retrieve specific packets at the given rate
   */
  typedef std::vector<std::pair<packet_id_e, uint16_t>> KvhPacketRequest;

  class KvhPacketStorage
  {

  private:
    KvhPacketMap packetMap_; ///< Map this class encapsulates. Maps packet id
    ///< to a pair with a bool for if the packet is updated, and a pointer
    ///< to the packet.

  public:
    KvhPacketStorage();

    /**
     * @code
     * KvhPacketStorage packetStorage;
     * packetStorage.Init(initIdsVector);
     * @endcode
     */
    int Init(KvhPacketRequest &);

    /**
     * @fn UpdatePacket
     * @param _packetId The id of the packet we want to update
     * @param _packetData A struct with data we want to update our storage to
     * @return int
     *  0 = Success
     *  -1 = Passed in packet type is not same as packet id
     *  -2 = Map does not contain this packet type currently
     * 
     * @code
     * KvhPacketStorage packetStorage;
     * packetStorage.Init(initIdsVector);
     * packet_type_t packet;
     * // Fill in data
     * packetStorage.updatePacket(packet_id, packet);
     * @endcode
     */
    template <class T>
    int UpdatePacket(packet_id_e _packetId, T& _packetData)
    {
      // Check if packet type matches packet id
      if (supportedPackets_.count(_packetId) <= 0 || packetTypeStr_[_packetId] != typeid(T).name())
      {
        return -1;
      }
      else if (this->Contains(_packetId))
      {
        *static_cast<T*>(packetMap_[_packetId].second.get()) = _packetData;
        return 0;
      }

      return -2;
    }
    
    /**
     * @code
     * // Create and init packetStorage
     * packetStorage.SetPacketUpdated(packetId, true);
     * @endcode
     */
    int SetPacketUpdated(packet_id_e, bool);

    /**
     * @code
     * if (packetStorage.PacketIsUpdated(packetId))
     *  // Do stuff
     * @endcode
     */
    bool PacketIsUpdated(packet_id_e);

    /**
     * @code
     * // Create and init packet storage
     * int error = packetStorage.GetPacket(packetId, packetStruct)
     * if (error == 0) // Do stuff
     * @endcode
     */
    template <class T>
    int GetPacket(packet_id_e _packetId, T& _packet)
    {
      // Check if packet types matches id
      if (supportedPackets_.count(_packetId) <= 0 || packetTypeStr_[_packetId] != typeid(T).name())
      {
        return -1;
      }
      else if (this->Contains(_packetId))
      {
        _packet = *static_cast<T*>(packetMap_[_packetId].second.get());
        return 0;
      }

      return -2;
    }

    /**
     * @code
     * // Create and init packet storage
     * if (packetStorage.Contains(packetId))
     *  // Do stuff
     * @endcode
     */
    bool Contains(packet_id_e);

    /**
     * @code
     * // Create and init packet storage
     * int size = packetStorage.Size();
     * @endcode
     */
    int Size();

    /**
     * @code
     * kvh::KvhPacketStorage::PrintPacketTypes();
     * @endcode
     */
    static void PrintPacketTypes();

    /**
     * @code
     * kvh::KvhPacketStorage::PrintPacketSize();
     * @endcode
     */
    static void PrintPacketSizes();
  };
} // namespace kvh
