#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/realtime_heave_surge_sway.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class RealTimeHeaveSurgeSway : public MemoryBlockParser
{
public:
    RealTimeHeaveSurgeSway() : MemoryBlockParser(2, 16) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::RealTimeHeaveSurgeSway res;
        buffer >> res.rt_heave_withoutBdL >> res.rt_heave_atBdL >> res.rt_surge_atBdL >>
            res.rt_sway_atBdL;
        outBinaryNav.rtHeaveSurgeSway = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder