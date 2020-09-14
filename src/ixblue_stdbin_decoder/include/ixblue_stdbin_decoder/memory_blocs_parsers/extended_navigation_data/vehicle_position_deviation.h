#pragma once

#include "ixblue_stdbin_decoder/data_models/extended_navigation_data/vehicle_position_deviation.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class VehiclePositionDeviation : public MemoryBlockParser
{
public:
    VehiclePositionDeviation() : MemoryBlockParser(6, 16) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::VehiclePositionDeviation res;
        buffer >> res.north_stddev_m >> res.east_stddev_m >> res.north_east_corr >>
            res.altitude_stddev_m;
        outBinaryNav.vehiclePositionDeviation = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder