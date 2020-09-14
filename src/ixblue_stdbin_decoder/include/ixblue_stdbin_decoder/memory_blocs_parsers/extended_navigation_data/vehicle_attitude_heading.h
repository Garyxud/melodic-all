#pragma once

#include "ixblue_stdbin_decoder/data_models/extended_navigation_data/vehicle_attitude_heading.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class VehicleAttitudeHeading : public MemoryBlockParser
{
public:
    VehicleAttitudeHeading() : MemoryBlockParser(3, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::VehicleAttitudeHeading res;
        buffer >> res.heading_deg >> res.roll_deg >> res.pitch_deg;
        outBinaryNav.vehicleAttitudeHeading = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder
