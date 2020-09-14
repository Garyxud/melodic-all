#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/system_date.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class SystemDate : public MemoryBlockParser
{
public:
    SystemDate() : MemoryBlockParser(13, 4) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::SystemDate res;
        buffer >> res.day >> res.month >> res.year;
        outBinaryNav.systemDate = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder