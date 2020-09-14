#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/current_geographic_frame.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class CurrentGeographicFrame : public MemoryBlockParser
{
public:
    CurrentGeographicFrame() : MemoryBlockParser(11, 8) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::CurrentGeographicFrame res;
        buffer >> res.north_msec >> res.east_msec;
        outBinaryNav.currentGeographicFrame = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder