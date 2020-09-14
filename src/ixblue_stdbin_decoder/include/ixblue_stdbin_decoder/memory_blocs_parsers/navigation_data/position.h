#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/position.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Position : public MemoryBlockParser
{
public:
    Position() : MemoryBlockParser(7, 21) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Position res;
        buffer >> res.latitude_deg >> res.longitude_deg >> res.altitude_ref >>
            res.altitude_m;
        outBinaryNav.position = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder