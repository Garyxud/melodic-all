#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/attitude_quaternion.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class AttitudeQuaternion : public MemoryBlockParser
{
public:
    AttitudeQuaternion() : MemoryBlockParser(26, 16) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::AttitudeQuaternion res;
        buffer >> res.q0 >> res.q1 >> res.q2 >> res.q3;
        outBinaryNav.attitudeQuaternion = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder