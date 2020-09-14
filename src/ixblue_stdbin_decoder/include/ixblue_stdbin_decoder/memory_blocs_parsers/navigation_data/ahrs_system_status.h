#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/ahrs_system_status.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class AHRSSystemStatus : public MemoryBlockParser
{
public:
    AHRSSystemStatus() : MemoryBlockParser(19, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::AHRSSystemStatus res;
        buffer >> res.status1 >> res.status2 >> res.status3;
        outBinaryNav.ahrsSystemStatus = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder