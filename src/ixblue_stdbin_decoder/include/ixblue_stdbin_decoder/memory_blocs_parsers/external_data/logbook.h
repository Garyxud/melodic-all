#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/logbook.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class LogBook : public MemoryBlockParser
{
public:
    LogBook() : MemoryBlockParser(27, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::LogBook res;
        buffer >> res.validityTime_100us >> res.log_id >> res.custom_text;
        outBinaryNav.logBook = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder