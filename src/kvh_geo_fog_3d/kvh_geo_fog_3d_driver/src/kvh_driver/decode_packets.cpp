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
 * @file decode_packets.cpp
 * @brief KVH Geo Fog 3D packet decoder
 * @author Trevor Bostic
 *
 * Leverages the AN packet library to convert to our internal structures
 */

#include "kvh_geo_fog_3d_driver.hpp"
#include <cmath>

#define RADIANS_TO_DEGREES (180.0 / M_PI)

namespace kvh
{

  /**
   * @fn Driver::DecodeUtmFix
   * @param _utmPacket [out] Our corrected utm packet
   * @param _anPacket [in] The an packet to decode
   * 
   * @brief The current api given by kvh incorrectly deals with this packet
   * so we needed to write our own decoder to capture all of the data. The utm_fix
   * struct type is extended from utm_packet to include the missing zone information
   * 
   * @return [int]:
   *  0 = Success
   *  -1 = Failure, packet was not of type utm, or not correct length
   */
  int Driver::DecodeUtmFix(utm_fix *_utmPacket, an_packet_t *_anPacket)
  {
    if (_anPacket->id == packet_id_utm_position && _anPacket->length == 26)
    {
      memcpy(&_utmPacket->position, &_anPacket->data[0], 3 * sizeof(double));
      _utmPacket->zone_num = _anPacket->data[24];
      _utmPacket->zone = _anPacket->data[25];
      return 0;
    }
    else
      return -1;
  }

  /**
   * @fn Driver::DecodePacket
   * @param _anPacket [in] The packet to decode into a Kvh packet type
   * 
   * @return [int]:
   *    0 = success,
   *    -1 = Not in list of currently stored packets
   *    -2 = Unable to decode packet
   * 
   * \todo replace if(debug) statements with logging
   * \todo Should decode packets be moved out of driver?
   */
  int Driver::DecodePacket(an_packet_t *_anPacket)
  {

    // See if packet id is in our map
    if (!(packetStorage_.Contains(static_cast<packet_id_e>(_anPacket->id))))
    {
      // If packet is not in our map, print out the id and length and return as unsupported
      if (debug_)
        printf("Decoding error: packet ID %u of Length %u\n", _anPacket->id, _anPacket->length);
      return -1;
    }

    /*********************************
     * PATTERN CODE NOTE - All code here follows the same pattern.
     * We shouldn't have to check for error in update or setupdated functions since
     * we confirm the packet is already available above.
     * Switch (packet_id) to determine packet type
     * case packet_id:
     *  if (decode(packet))
     *      update packet
     *      set packet update to true
     *  else
     *      return error unable to decode
     */

    switch (_anPacket->id)
    {
      case packet_id_system_state:
        // copy all the binary data into the typedef struct for the packet
        // this allows easy access to all the different values
        system_state_packet_t sysPacket;
        if (decode_system_state_packet(&sysPacket, _anPacket) == 0)
        {
          // Notify that we have updated packet
          packetStorage_.UpdatePacket(packet_id_system_state, sysPacket);
          packetStorage_.SetPacketUpdated(packet_id_system_state, true);

          if (debug_)
          {
            printf("Recieved system state packet.\n");
          }
        }
        else
        {
          if (debug_)
            printf("Failed to decode system state packet properly.\n");

          return -2;
        }
        break;
      case packet_id_unix_time:
        unix_time_packet_t unixPacket;
        if (decode_unix_time_packet(&unixPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_unix_time, unixPacket);
          packetStorage_.SetPacketUpdated(packet_id_unix_time, true);

          if (debug_)
          {
            printf("Recieved unix time packet.\n");
          }
        }
        else
        {
          if (debug_)
            printf("Failed to decode unix time packet properly.\n");

          return -2;
        }
        break;
      case packet_id_raw_sensors:
        raw_sensors_packet_t rawSensors;
        if (decode_raw_sensors_packet(&rawSensors, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_raw_sensors, rawSensors);
          packetStorage_.SetPacketUpdated(packet_id_raw_sensors, true);

          if (debug_)
          {
            printf("Recieved raw sensors packet.\n");
          }
        }
        else
        {
          if (debug_)
            printf("Failed to decode raw sensors packet properly.\n");

          return -2;
        }
        break;
      case packet_id_satellites:
        satellites_packet_t satellitesPacket;
        if (decode_satellites_packet(&satellitesPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_satellites, satellitesPacket);
          packetStorage_.SetPacketUpdated(packet_id_satellites, true);

          if (debug_)
            printf("Collected satellites packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode satellites packet properly.\n");

          return -2;
        }
        break;
      case packet_id_satellites_detailed:
        detailed_satellites_packet_t detailedPacket;
        if (decode_detailed_satellites_packet(&detailedPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_satellites_detailed, detailedPacket);
          packetStorage_.SetPacketUpdated(packet_id_satellites_detailed, true);

          if (debug_)
            printf("Collected detailed satellites packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode detailed satellites packet properly.\n");

          return -2;
        }
        break;
      case packet_id_local_magnetics:
        local_magnetics_packet_t magPacket;
        if (decode_local_magnetics_packet(&magPacket, _anPacket) == 0)
        {         
          packetStorage_.UpdatePacket(packet_id_local_magnetics, magPacket);
          packetStorage_.SetPacketUpdated(packet_id_local_magnetics, true);

          if (debug_)
            printf("Collected local magnetics packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode local magnetics packet properly.\n");

          return -2;
        }
        break;
      case packet_id_utm_position:
        // Below we had to create a modified decode function since

        utm_fix utmPacket;
        if (DecodeUtmFix(&utmPacket, _anPacket) == 0)
        {
          if (debug_) printf("UTM: %f, %f, %f\n", utmPacket.position[0], utmPacket.position[1], utmPacket.position[2]);
          packetStorage_.UpdatePacket(packet_id_utm_position, utmPacket);
          packetStorage_.SetPacketUpdated(packet_id_utm_position, true);

          if (debug_)
            printf("Collected utm position packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode utm position packet properly.\n");

          return -2;
        }
        break;
      case packet_id_ecef_position:
        ecef_position_packet_t ecefPacket;
        if (decode_ecef_position_packet(&ecefPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_ecef_position, ecefPacket);
          packetStorage_.SetPacketUpdated(packet_id_ecef_position, true);

          if (debug_)
            printf("Collected ecef position packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode ecef position packet properly.\n");

          return -2;
        }
        break;
      case packet_id_north_seeking_status:
        north_seeking_status_packet_t northPacket;
        if (decode_north_seeking_status_packet(&northPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_north_seeking_status, northPacket);
          packetStorage_.SetPacketUpdated(packet_id_north_seeking_status, true);

          if (debug_)
            printf("Collected north seeking status packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode north seeking status packet properly.\n");

          return -2;
        }
        break;
      case packet_id_euler_orientation_standard_deviation:
        euler_orientation_standard_deviation_packet_t eulerPacket;
        if (decode_euler_orientation_standard_deviation_packet(&eulerPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_euler_orientation_standard_deviation, eulerPacket);
          packetStorage_.SetPacketUpdated(packet_id_euler_orientation_standard_deviation, true);

          if (debug_)
            printf("Collected euler orientation standard deviation packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode euler orientation standard devation packet.\n");
          return -2;
        }
        break;
      case packet_id_odometer_state:
        odometer_state_packet_t odometerStatePacket;
        if (decode_odometer_state_packet(&odometerStatePacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_odometer_state, odometerStatePacket);
          packetStorage_.SetPacketUpdated(packet_id_odometer_state, true);

          if (debug_)
            printf("Collected odometer state packet.\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode odometer state packet.\n");
          return -2;
        }
        break;
      case packet_id_raw_gnss:
        raw_gnss_packet_t rawGnssPacket;
        if (decode_raw_gnss_packet(&rawGnssPacket, _anPacket) == 0)
        {
          packetStorage_.UpdatePacket(packet_id_raw_gnss, rawGnssPacket);
          packetStorage_.SetPacketUpdated(packet_id_raw_gnss, true);

          if (debug_)
            printf("Collected raw gnss packet\n");
        }
        else
        {
          if (debug_)
            printf("Failed to decode raw gnss packet\n");
          return -2;
        }
        break;
      default:
        break;
    }

    return 0;
  } // END DecodePacket()
} // namespace kvh
