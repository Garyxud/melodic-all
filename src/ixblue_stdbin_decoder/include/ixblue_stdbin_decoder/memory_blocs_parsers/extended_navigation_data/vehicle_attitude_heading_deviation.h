#pragma once

#include "ixblue_stdbin_decoder/data_models/extended_navigation_data/vehicle_attitude_heading_deviation.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class VehicleAttitudeHeadingDeviation : public MemoryBlockParser
{
public:
    VehicleAttitudeHeadingDeviation() : MemoryBlockParser(4, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::VehicleAttitudeHeadingDeviation res;
        buffer >> res.heading_stddev_deg >> res.roll_stddev_deg >> res.pitch_stddev_deg;
        outBinaryNav.vehicleAttitudeHeadingDeviation = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder