#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/acceleration_geographic_frame.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class AccelerationGeographicFrame : public MemoryBlockParser
{
public:
    AccelerationGeographicFrame() : MemoryBlockParser(23, 12) {}
    void parse(boost::asio::const_buffer& buffer, Data::BinaryNav& outBinaryNav) override
    {
        Data::AccelerationGeographicFrame res;
        buffer >> res.north_msec2 >> res.east_msec2 >> res.up_msec2;
        outBinaryNav.accelerationGeographicFrame = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder
