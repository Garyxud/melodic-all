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

#include "kvh_geo_fog_3d_device_configuration.hpp"

// RS 232
#include "rs232.h"

namespace kvh
{

  /**
   * @fn KvhDeviceConfig::CreatePacketPeriodsPacket
   * @param [in] _packetsRequested Vector of the packets requested and their associated frequencies
   * @param [out] _packetPeriods A correctly constructed packet periods packet
   * 
   * @return int
   *  0 = Success
   *  >0 = Number of unsupported or duplicated packets
   *  <0 = Error: Too many packets requested
   * 
   * PACKET RATE -> PACKET PERIOD
   * Formula for how packet frequency relates to packet period:
   * Packet Rate = 1000000/(Packet Period * Packet Timer Period)Hz - From Manual
   * 
   * Packet Timer Period can also be set, but we will use its default of 1000us for now
   * So, given packet rate, the packet period we need to input is
   * 
   * 1000000/(Packet Rate * 1000) = Packet Period
   * 1000/Packet Rate = Packet Period
   */
  int KvhDeviceConfig::CreatePacketPeriodsPacket(KvhPacketRequest &_packetsRequested, packet_periods_packet_t &_packetPeriods)
  {
    if (_packetsRequested.size() > MAXIMUM_PACKET_PERIODS)
    {
      return -1;
    }
    // Make permanent in case it has a hot reset, otherwise an error is likely
    _packetPeriods.permanent = 1;
    // Clear all exisiting packet periods and replace with new ones
    _packetPeriods.clear_existing_packets = 1;

    std::set<packet_id_e> packetIdList; // To hold list of already added packet id's
    // \todo Figure out what to do with duplicated and unsupported variables
    int duplicated = 0;
    int unsupported = 0;

    for (int i = 0; i < _packetsRequested.size(); i++)
    {
      std::pair<packet_id_e, int> packet = _packetsRequested[i];

      if (supportedPackets_.count(packet.first) == 0)
      {
        unsupported += 1;
        continue;
      }

      if (packetIdList.count(packet.first) > 0)
      {
        duplicated += 1; // Found duplicate, increase counter
        continue;
      }

      // packet_period_t period = {packet id, period}
      packet_period_t period;
      period.packet_id = _packetsRequested[i].first;
      period.period = 1000 / _packetsRequested[i].second; // Using formula for rate to period derived above
      _packetPeriods.packet_periods[i] = period;

      // Record we have seen this packet id to look for duplicates
      packetIdList.insert(packet.first);
    }

    return unsupported + duplicated;
  }

  /**
   * @fn KvhDeviceConfig::CreateFilterOptionsPacket
   * @param [out] _filterOptions The created filter options packet
   * @param [in] _permanent Whether the options should persist through reset. Default true
   * @param [in] _vehicle_type Determines the type of the vehicle. Used in internal kalman filter.
   * Default: vehicle_type_car, range 0-13
   * @param [in] _internal_gnss_enabled Whether gnss capabilities are enabled. Default true
   * @param [in] _atmospheric_altitude_enabled Whether atmospheric altitude capabilitites are enabled. Default true
   * @param [in] _velocity_heading_enabled Whether velocity heading is enabled. Default true
   * @param [in] _reversing_detection_enabled Whether detecting reverse motion is enabled. Default true
   * @param [in] _motion_analysis_enabled Whether motion analysis is enabled. Default true
   * 
   * @return int
   *  0 = Success
   *  -1 = Vehicle type out of range
   */
  int KvhDeviceConfig::CreateFilterOptionsPacket(
    filter_options_packet_t &_filterOptions,
    bool _permanent,
    uint8_t _vehicle_type,
    bool _internal_gnss_enabled,
    bool _atmospheric_altitude_enabled,
    bool _velocity_heading_enabled,
    bool _reversing_detection_enabled,
    bool _motion_analysis_enabled)
  {
    if (_vehicle_type > 13)
    {
      return -1;
    }

    _filterOptions.permanent = _permanent;
    _filterOptions.vehicle_type = _vehicle_type;
    _filterOptions.internal_gnss_enabled = _internal_gnss_enabled; // Set if we want to test using gps or not
    _filterOptions.reserved = 0;                                   // Need to set to 0, have found problems with other packets not doing this
    _filterOptions.atmospheric_altitude_enabled = _atmospheric_altitude_enabled;
    _filterOptions.velocity_heading_enabled = _velocity_heading_enabled;
    _filterOptions.reversing_detection_enabled = _reversing_detection_enabled;
    _filterOptions.motion_analysis_enabled = _motion_analysis_enabled;

    return 0;
  }
 
  /**
   * @fn KvhDeviceConfig::SetBaudRate
   * @param [in] _port The port connected to the kvh (Ex. /dev/ttyS0)
   * @param [in] _curBaudRate The current baud rate, required to connect and change baud rate
   * @param [in] _primaryBaudRate The rate to use for the primary RS-422 communications
   * @param [in] _gpioBaudRate The rate to use for the GPIO pins
   * @param [in] _auxBaudRate The rate to use for the Auxiliary RS-232 port
   * 
   * @brief This function can be used to set the buad rate independent of the other functions
   * of the driver. In most cases baud will need to be set once at the beginning, and then 
   * left alone unless more packets are added.
   * 
   * \todo Add way to set gpio and auxiliary baud rates to their current values
   * \todo Figure out how this function may be tested better
   * 
   * @return int
   *  -1 = Failure to open port
   *  -2 = Failure to set baud rate
   */
  int KvhDeviceConfig::SetBaudRate(std::string _port, int _curBaudRate, int _primaryBaudRate, int _gpioBaudRate, int _auxBaudRate)
  {
    // Create the baud rate packet that we want to send
    baud_rates_packet_t baudRatePacket;
    baudRatePacket.permanent = 1;
    baudRatePacket.primary_baud_rate = _primaryBaudRate;
    baudRatePacket.gpio_1_2_baud_rate = _gpioBaudRate;
    baudRatePacket.auxiliary_baud_rate = _auxBaudRate;
    baudRatePacket.reserved = 0;

    an_packet_t *requestPacket = encode_baud_rates_packet(&baudRatePacket);

    char portArr[4096];
    strncpy(portArr, _port.c_str(), 4096);
    if (OpenComport(portArr, _curBaudRate) != 0)
    {
      return -1;
    }

    an_packet_encode(requestPacket);
    // Send AN packet via serial port
    if (!SendBuf(an_packet_pointer(requestPacket), an_packet_size(requestPacket)))
    {
      return -2;
    }
    an_packet_free(&requestPacket);
    requestPacket = nullptr;

    CloseComport();

    return 0;
  }

  /**
   * @fn KvhDeviceConfig::FindCurrentBaudRate
   * @param [in] _port The port on the computer the kvh is connected to (e.g. '/dev/ttyUSB0')
   * @return int The current baudrate of the kvh if found, -1 otherwise
   * 
   * @brief This function tries each possible setting of baudrates until it
   * either finds one that is receiving packets from the kvh, or runs out of
   * possibilities.
   */
  int KvhDeviceConfig::FindCurrentBaudRate(std::string _port, int _startingBaud)
  {
    std::vector<int> baudRates = {
      1200, 1800, 2400, 4800, 9600,
      19200, 57600, 115200, 230400,
      460800, 500000, 576000, 921600, 1000000};

    char portArr[4096];
    strncpy(portArr, _port.c_str(), 4096);

    for (int baudRate : baudRates)
    {
      // Skip over baud rates lower than what we want to start at
      if (baudRate < _startingBaud)
      {
        continue;
      }

      printf("Opening with port: %s, baudrate: %d...\n", portArr, baudRate);
      if (OpenComport(portArr, baudRate) != 0)
      {
        continue; // Try next baud rate
      }

      an_decoder_t anDecoder;
      an_packet_t *anPacket;
      an_decoder_initialise(&anDecoder);
 
      // Check for data over the next second
      for (int i = 0; i < 20; i++)
      {
        int bytesRec = 0;
      
        // Check if new packets have been sent
        if ((bytesRec = PollComport(an_decoder_pointer(&anDecoder), an_decoder_size(&anDecoder))) > 0)
        {
          an_decoder_increment(&anDecoder, bytesRec);
          // See if we can decode a packet
          while ((anPacket = an_packet_decode(&anDecoder)) != NULL)
          {
            // If so, we have found the correct baud rate
            printf("Found a packet\n");
            CloseComport();
            an_packet_free(&anPacket);
            printf("Found connection at %d baud\n", baudRate);
            return baudRate;
          }
        }
        usleep(10000);
      }
      CloseComport();
    }
    return -1;
  }

  /**
   * @fn KvhDeviceConfig::CalculateRequiredBaud
   * @param _packetsRequested [in] The list of selected packet id's and their associated frequencies
   * 
   * CALCULATING BAUDRATE Formula:
   * Data throughput = (packet_length + 5 (for fixed packet overhead)) * rate
   * Minimum baud = data throughput * 11
   * 
   * @attention The maximum allowable hz is 1000, any input above that will return an error code
   * 
   * @return int The required baud rate for the packets requested. 
   *  >= 0 = Success, calculated baud rate
   *  -1 = Unsupported packet entered
   *  -2 = Duplicate packat id's
   *  -3 = Packet frequency exceeds max hz
   */
  int KvhDeviceConfig::CalculateRequiredBaud(KvhPacketRequest &_packetsRequested)
  {
    std::set<packet_id_e> packetIdList;
    int dataThroughput = 0;

    for (int i = 0; i < _packetsRequested.size(); i++)
    {
      std::pair<packet_id_e, uint16_t> packet = _packetsRequested[i];

      // Check for duplicate packets or unsupported packet id's
      if (packetSize_.count(packet.first) == 0)
      {
        return -1;
      }
      else if (packetIdList.count(packet.first) > 0)
      {
        return -2;
      }

      // Max allowed hz is 1000
      if (packet.second > 1000)
      {
        return -3;
      }

      // Increase required baudrate by (struct_size + 5) * rate
      dataThroughput += (packetSize_[packet.first] + 5) * packet.second;
      packetIdList.insert(packet.first);
    }

    return dataThroughput * 11; // Minimum baud = Data throughput * 11
  }

} // namespace kvh
