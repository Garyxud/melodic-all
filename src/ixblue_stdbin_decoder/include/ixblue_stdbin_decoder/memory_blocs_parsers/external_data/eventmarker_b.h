#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/eventmarker.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class EventMarkerB : public MemoryBlockParser
{
public:
    EventMarkerB() : MemoryBlockParser(19, 9) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::EventMarker res;
        buffer >> res.validityTime_100us >> res.event_id >> res.event_count;
        outBinaryNav.eventMarkerB = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder