#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/attitude_heading.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class AttitudeHeading : public MemoryBlockParser
{
public:
    AttitudeHeading() : MemoryBlockParser(0, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::AttitudeHeading res;
        buffer >> res.heading_deg >> res.roll_deg >> res.pitch_deg;
        outBinaryNav.attitudeHeading = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder
