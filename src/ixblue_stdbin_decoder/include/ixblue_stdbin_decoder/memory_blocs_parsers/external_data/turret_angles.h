#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/turret_angles.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class TurretAngles : public MemoryBlockParser
{
public:
    TurretAngles() : MemoryBlockParser(24, 16) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::TurretAngles res;
        buffer >> res.validityTime_100us >> res.headingbearingdrift_angle_deg >>
            res.roll_deg >> res.elevationpitch_deg;
        outBinaryNav.turretAngles = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder