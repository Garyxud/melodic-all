#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/dvl_ground_speed.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class DvlGroundSpeed1 : public MemoryBlockParser
{
public:
    DvlGroundSpeed1() : MemoryBlockParser(10, 37) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::DvlGroundSpeed res;
        buffer >> res.validityTime_100us >> res.dvl_id >> res.xv1_groundspeed_ms >>
            res.xv2_groundspeed_ms >> res.xv3_groundspeed_ms >> res.dvl_speedofsound_ms >>
            res.dvl_altitude_m >> res.xv1_stddev_ms >> res.xv2_stddev_ms >>
            res.xv3_stddev_ms;
        outBinaryNav.dvlGroundSpeed1 = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder