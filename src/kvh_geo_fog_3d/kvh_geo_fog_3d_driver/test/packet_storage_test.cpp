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
#include "kvh_geo_fog_3d_packet_storage.hpp"
#include "init_vars.hpp"

TEST(PacketStorage, initZero)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::zeroRequest));
    EXPECT_EQ(0, packetStorage.Size());
}

TEST(PacketStorage, initSmallRequest)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(2, packetStorage.Size());
}

TEST(PacketStorage, initLargeRequest)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::largeRequest));
    EXPECT_EQ(8, packetStorage.Size());
}

TEST(PacketStorage, initDuplicate)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(-1, packetStorage.Init(KvhPackReqEnv::duplicateRequest));
}

TEST(PacketStorage, initUnsupported)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(-2, packetStorage.Init(KvhPackReqEnv::unsupportedRequest));
}

TEST(PacketStorage, updatePacket)
{
    kvh::KvhPacketStorage packetStorage;
    system_state_packet_t sysPacket;
    sysPacket.system_status.r = 10;
    sysPacket.filter_status.r = 27;
    sysPacket.unix_time_seconds = 123456;
    sysPacket.microseconds = 98765;
    sysPacket.latitude = 123.456;
    sysPacket.longitude = 456.789;
    sysPacket.height = 100100;
    sysPacket.velocity[0] = 0;
    sysPacket.velocity[1] = 1;
    sysPacket.velocity[2] = 2;
    sysPacket.body_acceleration[0] = 4;
    sysPacket.body_acceleration[1] = 4;
    sysPacket.body_acceleration[2] = 4;
    sysPacket.g_force = 10;
    sysPacket.orientation[0] = 0;
    sysPacket.orientation[1] = 1;
    sysPacket.orientation[2] = 2;
    sysPacket.angular_velocity[0] = 0;
    sysPacket.angular_velocity[1] = 1;
    sysPacket.angular_velocity[2] = 2;
    sysPacket.standard_deviation[0] = 0;
    sysPacket.standard_deviation[1] = 1;
    sysPacket.standard_deviation[2] = 2;

    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(0, packetStorage.UpdatePacket(packet_id_system_state, sysPacket));
}

TEST(PacketStorage, updateIncorrectType)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    system_state_packet_t sysPacket;
    sysPacket.height = 32;

    EXPECT_EQ(-1, packetStorage.UpdatePacket(packet_id_raw_sensors, sysPacket));
}

TEST(PacketStorage, updateUncontained)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    kvh::utm_fix utmPacket;
    utmPacket.position[0] = 12;

    EXPECT_EQ(-2, packetStorage.UpdatePacket(packet_id_utm_position, utmPacket));
}

TEST(PacketStorage, updateUnsupportedandUncontained)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    wind_packet_t windPacket;
    EXPECT_EQ(-1, packetStorage.UpdatePacket(packet_id_wind, windPacket));
}

TEST(PacketStorage, setPacketUpdatedandPacketIsUpdated)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    system_state_packet_t sysPacket;
    sysPacket.height = 32;

    // Update packet and set to true
    EXPECT_EQ(0, packetStorage.UpdatePacket(packet_id_system_state, sysPacket));
    EXPECT_EQ(0, packetStorage.SetPacketUpdated(packet_id_system_state, true));
    EXPECT_EQ(true, packetStorage.PacketIsUpdated(packet_id_system_state));

    EXPECT_EQ(0, packetStorage.SetPacketUpdated(packet_id_system_state, false));
    EXPECT_EQ(false, packetStorage.PacketIsUpdated(packet_id_system_state));
}

TEST(PacketStorage, packetIsUpdatedNotInit)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(false, packetStorage.PacketIsUpdated(packet_id_system_state));
}

TEST(PacketStorage, setPacketUpdatedUncontained)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(-1, packetStorage.SetPacketUpdated(packet_id_wind, true));
}

TEST(PacketStorage, packetIsUpdatedUncontained)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(false, packetStorage.PacketIsUpdated(packet_id_wind));
}

TEST(PacketStorage, getPacket)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    system_state_packet_t sysPacket;
    EXPECT_EQ(0, packetStorage.GetPacket(packet_id_system_state, sysPacket));
}

// \todo Make sure getPacketNotInit test works
TEST(PacketStorage, getPacketNotInit)
{
    kvh::KvhPacketStorage packetStorage;
    system_state_packet_t sysPacket;
    EXPECT_EQ(-2, packetStorage.GetPacket(packet_id_system_state, sysPacket));
}

TEST(PacketStorage, getPacketIncorrectType)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    wind_packet_t windPacket;
    EXPECT_EQ(-1, packetStorage.GetPacket(packet_id_utm_position, windPacket));
}

TEST(PacketStorage, getPacketUncontained)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    kvh::utm_fix utmPacket;
    EXPECT_EQ(-2, packetStorage.GetPacket(packet_id_utm_position, utmPacket));
}

TEST(PacketStorage, getPacketUnsupportedandUncontained)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    wind_packet_t windPacket;
    EXPECT_EQ(-1, packetStorage.GetPacket(packet_id_wind, windPacket));
}

TEST(PacketStorage, updatePacketandGetPacket)
{
    kvh::KvhPacketStorage packetStorage;
    system_state_packet_t sysPacket;
    sysPacket.system_status.r = 10;
    sysPacket.filter_status.r = 27;
    sysPacket.unix_time_seconds = 123456;
    sysPacket.microseconds = 98765;
    sysPacket.latitude = 123.456;
    sysPacket.longitude = 456.789;
    sysPacket.height = 100100;
    sysPacket.velocity[0] = 0;
    sysPacket.velocity[1] = 1;
    sysPacket.velocity[2] = 2;
    sysPacket.body_acceleration[0] = 4;
    sysPacket.body_acceleration[1] = 4;
    sysPacket.body_acceleration[2] = 4;
    sysPacket.g_force = 10;
    sysPacket.orientation[0] = 0;
    sysPacket.orientation[1] = 1;
    sysPacket.orientation[2] = 2;
    sysPacket.angular_velocity[0] = 0;
    sysPacket.angular_velocity[1] = 1;
    sysPacket.angular_velocity[2] = 2;
    sysPacket.standard_deviation[0] = 0;
    sysPacket.standard_deviation[1] = 1;
    sysPacket.standard_deviation[2] = 2;

    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(0, packetStorage.UpdatePacket(packet_id_system_state, sysPacket));
    system_state_packet_t sysPacket2;
    EXPECT_EQ(0, packetStorage.GetPacket<system_state_packet_t>(packet_id_system_state, sysPacket2));

    // Compare structs
    EXPECT_EQ(sysPacket.system_status.r, sysPacket2.system_status.r);
    EXPECT_EQ(sysPacket.filter_status.r, sysPacket2.filter_status.r);
    EXPECT_EQ(sysPacket.unix_time_seconds, sysPacket2.unix_time_seconds);
    EXPECT_EQ(sysPacket.microseconds, sysPacket2.microseconds);
    EXPECT_EQ(sysPacket.latitude, sysPacket2.latitude);
    EXPECT_EQ(sysPacket.longitude, sysPacket2.longitude);
    EXPECT_EQ(sysPacket.height, sysPacket2.height);
    EXPECT_EQ(sysPacket.velocity[0], sysPacket2.velocity[0]);
    EXPECT_EQ(sysPacket.velocity[1], sysPacket2.velocity[1]);
    EXPECT_EQ(sysPacket.velocity[2], sysPacket2.velocity[2]);
    EXPECT_EQ(sysPacket.body_acceleration[0], sysPacket2.body_acceleration[0]);
    EXPECT_EQ(sysPacket.body_acceleration[1], sysPacket2.body_acceleration[1]);
    EXPECT_EQ(sysPacket.body_acceleration[2], sysPacket2.body_acceleration[2]);
    EXPECT_EQ(sysPacket.g_force, sysPacket2.g_force);
    EXPECT_EQ(sysPacket.orientation[0], sysPacket2.orientation[0]);
    EXPECT_EQ(sysPacket.orientation[1], sysPacket2.orientation[1]);
    EXPECT_EQ(sysPacket.orientation[2], sysPacket2.orientation[2]);
    EXPECT_EQ(sysPacket.angular_velocity[0], sysPacket2.angular_velocity[0]);
    EXPECT_EQ(sysPacket.angular_velocity[1], sysPacket2.angular_velocity[1]);
    EXPECT_EQ(sysPacket.angular_velocity[2], sysPacket2.angular_velocity[2]);
    EXPECT_EQ(sysPacket.standard_deviation[0], sysPacket2.standard_deviation[0]);
    EXPECT_EQ(sysPacket.standard_deviation[1], sysPacket2.standard_deviation[1]);
    EXPECT_EQ(sysPacket.standard_deviation[2], sysPacket2.standard_deviation[2]);
}

// This test is included in addtion to the one above since we needed to make
// modifications to the utm packet. 
TEST(PacketStorage, UTM_UpdateAndGetPacket)
{
    kvh::KvhPacketStorage packetStorage;
    kvh::utm_fix utmPacket;
    utmPacket.position[0] = 32;
    utmPacket.position[1] = 42;
    utmPacket.position[2] = 133;
    utmPacket.zone = 'B';
    utmPacket.zone_num = 12;

    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::largeRequest));
    EXPECT_EQ(0, packetStorage.UpdatePacket(packet_id_utm_position, utmPacket));
    kvh::utm_fix utmPacket2;
    EXPECT_EQ(0, packetStorage.GetPacket<kvh::utm_fix>(packet_id_utm_position, utmPacket2));

    // Compare structs
    EXPECT_EQ(utmPacket.position[0], utmPacket2.position[0]);
    EXPECT_EQ(utmPacket.position[1], utmPacket2.position[1]);
    EXPECT_EQ(utmPacket.position[2], utmPacket2.position[2]);
    EXPECT_EQ(utmPacket.zone, utmPacket2.zone);
    EXPECT_EQ(utmPacket.zone_num, utmPacket2.zone_num);
}

TEST(PacketStorage, containsTrue)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(true, packetStorage.Contains(packet_id_system_state));
    EXPECT_EQ(true, packetStorage.Contains(packet_id_raw_sensors));
}

TEST(PacketStorage, containsFalse)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(false, packetStorage.Contains(packet_id_wind));
}

TEST(PacketStorage, sizeSmall)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::smallRequest));
    EXPECT_EQ(2, packetStorage.Size());
}

TEST(PacketStorage, sizeLarge)
{
    kvh::KvhPacketStorage packetStorage;
    EXPECT_EQ(0, packetStorage.Init(KvhPackReqEnv::largeRequest));
    EXPECT_EQ(8, packetStorage.Size());
}

TEST(PacketStorage, printPacketTypes)
{
    EXPECT_NO_FATAL_FAILURE(kvh::KvhPacketStorage::PrintPacketTypes());
}

TEST(PacketStorage, printPacketSizes)
{
    EXPECT_NO_FATAL_FAILURE(kvh::KvhPacketStorage::PrintPacketSizes());
}
