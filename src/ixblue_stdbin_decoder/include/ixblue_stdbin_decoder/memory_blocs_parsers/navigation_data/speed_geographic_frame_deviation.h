#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/speed_geographic_frame_deviation.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class SpeedGeographicFrameDeviation : public MemoryBlockParser
{
public:
    SpeedGeographicFrameDeviation() : MemoryBlockParser(10, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::SpeedGeographicFrameDeviation res;
        buffer >> res.north_stddev_msec >> res.east_stddev_msec >> res.up_stddev_msec;
        outBinaryNav.speedGeographicFrameDeviation = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder