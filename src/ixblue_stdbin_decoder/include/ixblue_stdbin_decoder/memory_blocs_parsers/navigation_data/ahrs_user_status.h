#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/ahrs_user_status.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class AHRSUserStatus : public MemoryBlockParser
{
public:
    AHRSUserStatus() : MemoryBlockParser(20, 4) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::AHRSUserStatus res;
        buffer >> res.status;
        outBinaryNav.ahrsUserStatus = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder