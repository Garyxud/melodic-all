#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/heading_roll_pitch_rate.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class HeadingRollPitchRate : public MemoryBlockParser
{
public:
    HeadingRollPitchRate() : MemoryBlockParser(4, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::HeadingRollPitchRate res;
        buffer >> res.heading_rate >> res.roll_rate >> res.pitch_rate;
        outBinaryNav.headingRollPitchRate = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder