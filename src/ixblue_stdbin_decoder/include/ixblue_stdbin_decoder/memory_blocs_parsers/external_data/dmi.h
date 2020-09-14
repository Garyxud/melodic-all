#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/dmi.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Dmi : public MemoryBlockParser
{
public:
    Dmi() : MemoryBlockParser(13, 8) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Dmi res;
        buffer >> res.validityTime_100us >> res.pulse_count;
        outBinaryNav.dmi = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder