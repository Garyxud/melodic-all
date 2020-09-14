#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/speed_geographic_frame.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class SpeedGeographicFrame : public MemoryBlockParser
{
public:
    SpeedGeographicFrame() : MemoryBlockParser(9, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::SpeedGeographicFrame res;
        buffer >> res.north_msec >> res.east_msec >> res.up_msec;
        outBinaryNav.speedGeographicFrame = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder