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

#pragma once

#include <gtest/gtest.h>
#include "kvh_geo_fog_3d_driver.hpp"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

class KvhPackReqEnv : public ::testing::Environment
{
public:
    typedef std::pair<packet_id_e, uint16_t> freqPair;

    static kvh::KvhPacketRequest zeroRequest;
    static kvh::KvhPacketRequest smallRequest;
    static kvh::KvhPacketRequest largeRequest;
    static kvh::KvhPacketRequest largeVaryingRatesRequest;
    static kvh::KvhPacketRequest minFrequencyRequest;
    static kvh::KvhPacketRequest maxFrequencyRequest;
    static kvh::KvhPacketRequest exceedingMaxFrequencyRequest;
    static kvh::KvhPacketRequest duplicateRequest;
    static kvh::KvhPacketRequest unsupportedRequest;
    static kvh::KvhPacketRequest unsupportedDuplicateRequest;
};
