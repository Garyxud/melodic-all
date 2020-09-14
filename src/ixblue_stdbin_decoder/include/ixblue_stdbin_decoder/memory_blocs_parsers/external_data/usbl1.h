#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/usbl.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Usbl1 : public MemoryBlockParser
{
public:
    Usbl1() : MemoryBlockParser(6, 49) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Usbl res;
        buffer >> res.validityTime_100us >> res.usbl_id >> res.beacon_id >>
            res.latitude_deg >> res.longitude_deg >> res.altitude_m >>
            res.north_stddev_m >> res.east_stddev_m >> res.lat_lon_cov_m2 >>
            res.altitude_stddev_m;
        outBinaryNav.usbl1 = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder