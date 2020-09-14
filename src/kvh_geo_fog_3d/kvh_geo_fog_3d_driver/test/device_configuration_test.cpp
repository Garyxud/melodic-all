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

#include <gtest/gtest.h>
#include "kvh_geo_fog_3d_driver.hpp"
#include "kvh_geo_fog_3d_device_configuration.hpp"
#include "init_vars.hpp"

TEST(DeviceConfiguration, calcBaudRateZeroPacket){
    EXPECT_EQ(0, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::zeroRequest));
}

TEST(DeviceConfiguration, calcBaudRateSmall){
    EXPECT_EQ(89100, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::smallRequest));
}

TEST(DeviceConfiguration, calcBaudRateLarge){
    EXPECT_EQ(145497, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::largeRequest));
}

TEST(DeviceConfiguration, calcBaudRateLargeVarying){
    EXPECT_EQ(152196, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::largeVaryingRatesRequest));
}

TEST(DeviceConfiguration, calcBaudRateMinFrequency){
    EXPECT_EQ(0, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::minFrequencyRequest));
}

TEST(DeviceConfiguration, calcBaudRateMaxFrequency){
    EXPECT_EQ(1199000, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::maxFrequencyRequest));
}

// We should only allow up to 1000 Hz
TEST(DeviceConfiguration, calcBaudRateExceedingMaxFrequency){
    EXPECT_EQ(-3, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::exceedingMaxFrequencyRequest));
}

TEST(DeviceConfiguration, calcBaudRateUnsupported){
    EXPECT_EQ(-1, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::unsupportedRequest));
}

TEST(DeviceConfiguration, calcBaudRateDuplicate){
    EXPECT_EQ(-2, kvh::KvhDeviceConfig::CalculateRequiredBaud(KvhPackReqEnv::duplicateRequest));
}

TEST(DeviceConfiguration, createPacketPeriodsZeroRequest)
{
    packet_periods_packet_t periodsPacket;
    memset(&periodsPacket, 0, sizeof(periodsPacket));
    EXPECT_EQ(0, kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(
        KvhPackReqEnv::zeroRequest, periodsPacket));
    // No packets should have been added
    EXPECT_EQ(0, periodsPacket.packet_periods[0].packet_id);
}

TEST(DeviceConfiguration, createPacketPeriodsSmallRequest)
{
    packet_periods_packet_t periodsPacket;
    EXPECT_EQ(0, kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(
        KvhPackReqEnv::smallRequest, periodsPacket));
    // The periods below come from the formula in their tech document
    // period = 1000 / frequency
    // This is a simplified equations as you can technically change the 1000
    EXPECT_EQ(packet_id_system_state, periodsPacket.packet_periods[0].packet_id);
    EXPECT_EQ(20, periodsPacket.packet_periods[0].period);
    EXPECT_EQ(packet_id_raw_sensors, periodsPacket.packet_periods[1].packet_id);
    EXPECT_EQ(20, periodsPacket.packet_periods[1].period);
}

TEST(DeviceConfiguration, createPacketPeriodsUnsupported)
{
    packet_periods_packet_t periodsPacket;
    EXPECT_EQ(1, kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(
        KvhPackReqEnv::unsupportedRequest, periodsPacket));
}

TEST(DeviceConfiguration, createPacketPeriodsDuplicated)
{
    packet_periods_packet_t periodsPacket;
    EXPECT_EQ(1, kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(
        KvhPackReqEnv::duplicateRequest, periodsPacket));
}

TEST(DeviceConfiguration, createPacketPeriodsUnsupportedandDuplicated)
{
    packet_periods_packet_t periodsPacket;
    EXPECT_EQ(3, kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(
        KvhPackReqEnv::unsupportedDuplicateRequest, periodsPacket));
}

TEST(DeviceConfiguration, createPacketPeriodsExceedingMaxPeriods)
{
    kvh::KvhPacketRequest maxPeriods;
    for(int i = 0; i < 60; i++)
    {
        maxPeriods.push_back(KvhPackReqEnv::freqPair(packet_id_system_state, 20));
    }
    packet_periods_packet_t periodsPacket;
    EXPECT_EQ(-1, kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(
        maxPeriods, periodsPacket));
}

TEST(DeviceConfiguration, createFilterOptionsDefault)
{
    filter_options_packet_t filterOptions;
    EXPECT_EQ(0, kvh::KvhDeviceConfig::CreateFilterOptionsPacket(
        filterOptions));

    EXPECT_EQ(true, filterOptions.permanent);
    EXPECT_EQ(vehicle_type_car, filterOptions.vehicle_type);
    EXPECT_EQ(true, filterOptions.internal_gnss_enabled);
    EXPECT_EQ(true, filterOptions.atmospheric_altitude_enabled);
    EXPECT_EQ(true, filterOptions.velocity_heading_enabled);
    EXPECT_EQ(true, filterOptions.reversing_detection_enabled);
    EXPECT_EQ(true, filterOptions.motion_analysis_enabled);
}

TEST(DeviceConfiguration, createFilterOptionsVehicleOutOfRange)
{
    filter_options_packet_t filterOptions;
    EXPECT_EQ(-1, kvh::KvhDeviceConfig::CreateFilterOptionsPacket(
        filterOptions, true, 15));
}

TEST(DeviceConfiguration, setBaudRate)
{
    EXPECT_EQ(-1, kvh::KvhDeviceConfig::SetBaudRate("/dev/ttyUSB0", 115200, 230400));
}
