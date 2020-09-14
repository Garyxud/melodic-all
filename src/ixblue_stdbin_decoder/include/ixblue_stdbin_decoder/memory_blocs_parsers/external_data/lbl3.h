#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/lbl.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Lbl3 : public MemoryBlockParser
{
public:
    Lbl3() : MemoryBlockParser(16, 41) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Lbl res;
        buffer >> res.validityTime_100us >> res.rfu >> res.beacon_id >>
            res.beacon_latitude_deg >> res.beacon_longitude_deg >>
            res.beacon_altitude_m >> res.range_m >> res.range_stddev_m;
        outBinaryNav.lbl3 = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder