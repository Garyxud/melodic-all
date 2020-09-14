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
 * @file driver_main.cpp
 * @brief KVH Geo Fog 3D driver class definitions.
 * @author Trevor Bostic
 *
 * This file implements all functions defined in kvh_geo_fog_3d_driver.hpp. The driver 
 * is used for for interfacing with KVH GEO FOG over serial connection.
 */

//STD includes
#include <cstdio>
#include <string>
#include <cmath>
#include <functional>
#include <termios.h>
#include <typeinfo>
#include <set>
#include <iostream>
#include <fstream>
#include <unistd.h>

// RS232
#include "rs232.h"

// KVH Includes
#include "kvh_geo_fog_3d_driver.hpp"

namespace kvh
{

  /**
   * @fn Driver::Driver
   * @brief Initializes connected status, port to use, and if debug printing is turned on.
   * 
   * @param _debug [in] Determines if debug statements are printed.
   */
  Driver::Driver(bool _debug) : connected_(false),
                                port_("/dev/ttyUSB0"),
                                debug_(_debug)
  {
  } // END Driver()

  /**
   * @fn Driver::~Driver
   * @brief Destructor. Will automatically cleanup the driver.
   */
  Driver::~Driver()
  {
    Cleanup();
  } // END ~Driver()

  //////////////////////////
  // PRIVATE FUNCTIONS
  //////////////////////////

  /**
   * @fn Driver::SendPacket
   * @brief Wrapper function for more easily sending an packets via serial port
   * 
   * @param _anPacket [in] The an packet to transmit
   */
  int Driver::SendPacket(an_packet_t *_anPacket)
  {
    // Encode packet. Adds header including LRC and CRC
    an_packet_encode(_anPacket);
    // Send AN packet via serial port
    if (SendBuf(an_packet_pointer(_anPacket), an_packet_size(_anPacket)))
    {
      packetRequests_.push_back(static_cast<packet_id_e>(_anPacket->id));
      return 0;
    }
    else
    {
      return -1;
    }
  } // END SendPacket()

  //////////////////////////////////////////////
  // PUBLIC FUNCTIONS
  //////////////////////////////////////////////

  /**
   * @fn Driver::Init(const std::string& _port, KvhPacketRequest _packetsRequested)
   * @param [in] _port The port to connect to the kvh through
   * @param [in] _packetsRequested The requested packets and their associated frequencies
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
   * 
   * @brief This function will intialize with all of the default options. For more in depth
   * information see the the overlaoded init function.
   */
  int Driver::Init(const std::string &_port, KvhPacketRequest &_packetsRequested)
  {
    return Driver::Init(_port, _packetsRequested, defaultOptions_);
  }

  /**
   * @fn Driver::Init(const std::string& _port, KvhPacketRequest _packetsRequested, KvhInitOptions _initOptions)
   * @brief Initialize the connection to the device
   * 
   * @param _port [in] Port the kvh is connected through
   * @param _packetsRequested [in] Vector of packet id's to ask the kvh to output
   * @
   * 
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
   * 
   * Initialize the serial connection to the KVH GEO FOG 3D.
   * 
   * Current calculation for our packets:
   * (105 (sys state) + 18 (satellites) +
   * (5+(7*(1 for min or 50 for max))) (Detailed satellites) + 17 (local mag)
   * + 30 (utm) + 29 (ecef) + 32) * rate (50hz default) * 11
   * Minimum baud all packets at 100hz for worst case scenario is 644600
   */
  int Driver::Init(const std::string &_port, KvhPacketRequest &_packetsRequested, KvhInitOptions _initOptions)
  {
    int returnValue = 0;
    int errorCode;
    debug_ = _initOptions.debugOn;
    if (debug_)
      printf("Debug statements enabled.\n");

    if ((errorCode = packetStorage_.Init(_packetsRequested)) != 0)
    {
      if (debug_)
        printf("Unable to intialize packet storage. Error code: %d", errorCode);
      return -1;
    }
    ////////////////////////////////
    // CREATE PACKET PERIODS PACKET
    ////////////////////////////////

    packet_periods_packet_t packetPeriods;
    deviceConfig_.CreatePacketPeriodsPacket(_packetsRequested, packetPeriods);

    ////////////////////////////////
    // CALCULATING BUAD RATE
    ////////////////////////////////

    // dataThroughput from above, 11 from their equation
    int minBaudRequired = deviceConfig_.CalculateRequiredBaud(_packetsRequested);
    if (debug_)
      printf("Calculated required minimum baud rate: %d\n", minBaudRequired);

    // \todo Handle returned value appropriately
    if (minBaudRequired < _initOptions.baudRate)
    {
      returnValue = 1;
    }

    ///////////////////////////////////////
    // SETTING UP KALMAN FILTER OPTIONS
    ///////////////////////////////////////

    filter_options_packet_t filterOptions;
    if (deviceConfig_.CreateFilterOptionsPacket(filterOptions, true, _initOptions.filterVehicleType,
                                                _initOptions.gnssEnabled, _initOptions.atmosphericAltitudeEnabled, _initOptions.velocityHeadingEnabled,
                                                _initOptions.reversingDetectionEnabled, _initOptions.motionAnalysisEnabled) != 0)
    {
      return -2;
    }

    ////////////////////////////////////////
    // CONNECTING TO KVH
    ////////////////////////////////////////

    port_ = _port;
    char portArr[4096];
    strncpy(portArr, port_.c_str(), 4096);
    if (debug_) printf("Baud: %d\n", _initOptions.baudRate);
    if (OpenComport(portArr, _initOptions.baudRate) != 0)
    {
      if (debug_)
        printf("Unable to establish connection.\n");
      return -3;
    }
    // We are connected to the KVH!
    connected_ = true;

    ////////////////////////////////
    // SENDING CONFIGURATION PACKETS
    ////////////////////////////////

    an_packet_t *requestPacket;
    int packetError;

    if (debug_)
      printf("Sending packet_periods.\n");

    requestPacket = encode_packet_periods_packet(&packetPeriods);
    packetError = SendPacket(requestPacket);
    an_packet_free(&requestPacket);
    requestPacket = nullptr;
    if (packetError)
    {
      return -4;
    }

    if (debug_)
      printf("Sending filter options packet.");

    requestPacket = encode_filter_options_packet(&filterOptions);
    packetError = SendPacket(requestPacket);
    requestPacket = nullptr;
    if (packetError != 0)
    {
      return -5;
    }

    /////////////////////////////////////
    // INITIALISE AN DECODER
    /////////////////////////////////////

    if (debug_)
      printf("Initializing decoder.\n");
    an_decoder_initialise(&anDecoder_);

    return returnValue;

  } // END Init()

  /**
   * @fn Driver::Once
   * @brief Do a single data read from the device
   * 
   * @return [int]:
   *   0  = success
   *   -1 = CRC16 failure
   *   -2 = LRC failure
   * 
   * Single data packet read.
   * 
   * \attention This function is a bit of a mess, due to how their api has a different function
   * for every single type of packet. Our goal is to be able to deal with all of them simply
   * within one single function, but that brings a lot of typing problems into the mix. Namely
   * making sure that we have the correct type of struct, and then calling the correct decoding
   * function for that struct.
   */
  int Driver::Once()
  {
    an_packet_t *anPacket;
    int bytesRec = 0;
    int unexpectedPackets = 0;

    // Check if new packets have been sent
    if ((bytesRec = PollComport(an_decoder_pointer(&anDecoder_), an_decoder_size(&anDecoder_))) > 0)
    {
      /* increment the decode buffer length by the number of bytes received */
      an_decoder_increment(&anDecoder_, bytesRec);

      /* decode all the packets in the buffer */
      while ((anPacket = an_packet_decode(&anDecoder_)) != NULL)
      {
        // If we get an acknowledgment packet from sending packets
        // Acknowledgement packet is different than the others so we keep it seperate
        if (anPacket->id == packet_id_acknowledge)
        {
          acknowledge_packet_t ackP;
          if (decode_acknowledge_packet(&ackP, anPacket) == 0)
          {
            if (debug_)
            {
              printf("*********************************\n"
                     "Acknowledging packet from packet id: %d\n Result of packet %d\n"
                     "********************************\n",
                     ackP.packet_id, ackP.acknowledge_result);
            }
          }
          else
          {
            if (debug_)
              printf("Unable to decode acknowledge packet properly.\n");
          }
        }
        else
        {
          if (DecodePacket(anPacket) < 0)
            unexpectedPackets++;
        }

        /* Ensure that you free the an_packet when your done with it or you will leak memory */
        an_packet_free(&anPacket);
      }
    }

    if (debug_) printf("Recieved %d unexpected packets during transmission.", unexpectedPackets);
  } // END Once()

  /**
   * @fn Driver::PacketIsUpdated
   * @param _packetId The id of the packet you are checking
   * @return True if the packet has been updated, false if not
   * 
   * @brief Use this function to determine if new packet data
   * has arrived since the last time you checked
   */
  bool Driver::PacketIsUpdated(packet_id_e _packetId)
  {
    return packetStorage_.PacketIsUpdated(_packetId);
  }

  /**
   * @fn Driver::SetPacketUpdated
   * @param _packetId The id of the packet you want to set the status of
   * @param _updateStatus The value you want to set the packet to.
   * 
   * @brief Use this function to set that the packet has been updated (though
   * the driver will usually do that itself), or use it to notify the driver
   * that you have used the most recent packet.
   */
  int Driver::SetPacketUpdated(packet_id_e _packetId, bool _updateStatus)
  {
    return packetStorage_.SetPacketUpdated(_packetId, _updateStatus);
  }

  /**
   * @fn Driver::Cleanup
   * @brief Cleanup and close our connections.
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
   */
  int Driver::Cleanup()
  {
    if (connected_)
      CloseComport();
    return 0;
  } // END Cleanup()

} // namespace kvh
