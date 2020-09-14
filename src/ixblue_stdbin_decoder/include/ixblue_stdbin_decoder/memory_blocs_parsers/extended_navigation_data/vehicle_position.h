#pragma once

#include "ixblue_stdbin_decoder/data_models/extended_navigation_data/vehicle_position.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class VehiclePosition : public MemoryBlockParser
{
public:
    VehiclePosition() : MemoryBlockParser(5, 21) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::VehiclePosition res;
        buffer >> res.latitude_deg >> res.longitude_deg >> res.altitude_ref >>
            res.altitude_m;
        outBinaryNav.vehiclePosition = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder