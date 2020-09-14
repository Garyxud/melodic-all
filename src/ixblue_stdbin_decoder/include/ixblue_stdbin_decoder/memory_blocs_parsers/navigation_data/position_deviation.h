#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/position_deviation.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class PositionDeviation : public MemoryBlockParser
{
public:
    PositionDeviation() : MemoryBlockParser(8, 16) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::PositionDeviation res;
        buffer >> res.north_stddev_m >> res.east_stddev_m >> res.north_east_corr >>
            res.altitude_stddev_m;
        outBinaryNav.positionDeviation = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder