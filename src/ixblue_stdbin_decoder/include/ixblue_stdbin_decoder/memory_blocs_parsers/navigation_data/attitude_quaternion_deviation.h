#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/attitude_quaternion_deviation.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class AttitudeQuaternionDeviation : public MemoryBlockParser
{
public:
    AttitudeQuaternionDeviation() : MemoryBlockParser(27, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::AttitudeQuaternionDeviation res;
        buffer >> res.quat_stddev_xi1 >> res.quat_stddev_xi2 >> res.quat_stddev_xi3;
        outBinaryNav.attitudeQuaternionDeviation = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder