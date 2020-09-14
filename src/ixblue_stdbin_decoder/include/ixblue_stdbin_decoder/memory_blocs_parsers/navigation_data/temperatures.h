#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/temperatures.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Temperatures : public MemoryBlockParser
{
public:
    Temperatures() : MemoryBlockParser(25, 12) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Temperatures res;
        buffer >> res.mean_temp_fog >> res.mean_temp_acc >> res.board_temperature;
        outBinaryNav.temperatures = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder