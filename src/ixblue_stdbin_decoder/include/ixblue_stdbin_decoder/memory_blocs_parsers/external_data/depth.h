#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/depth.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Depth : public MemoryBlockParser
{
public:
    Depth() : MemoryBlockParser(9, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Depth res;
        buffer >> res.validityTime_100us >> res.depth_m >> res.depth_stddev_m;
        outBinaryNav.depth = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder