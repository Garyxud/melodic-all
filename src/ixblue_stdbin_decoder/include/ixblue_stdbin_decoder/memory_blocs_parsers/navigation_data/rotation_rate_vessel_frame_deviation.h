#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/rotation_rate_vessel_frame_deviation.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class RotationRateVesselFrameDeviation : public MemoryBlockParser
{
public:
    RotationRateVesselFrameDeviation() : MemoryBlockParser(30, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::RotationRateVesselFrameDeviation res;
        buffer >> res.xv1_stddev_degsec >> res.xv2_stddev_degsec >> res.xv3_stddev_degsec;
        outBinaryNav.rotationRateVesselFrameDeviation = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder