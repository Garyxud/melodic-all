#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/eventmarker.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class EventMarker : public MemoryBlockParser
{
public:
    EventMarker(int bit_pose) : MemoryBlockParser(bit_pose, 9) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::EventMarker res;
        buffer >> res.validityTime_100us >> res.event_id >> res.event_count;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::EventMarker& res, Data::BinaryNav& outBinaryNav) = 0;
};

class EventMarkerA : public EventMarker
{
public:
    EventMarkerA() : EventMarker(18) {}

protected:
    void fillRes(const Data::EventMarker& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.eventMarkerA = res;
    }
};

class EventMarkerB : public EventMarker
{
public:
    EventMarkerB() : EventMarker(19) {}

protected:
    void fillRes(const Data::EventMarker& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.eventMarkerB = res;
    }
};

class EventMarkerC : public EventMarker
{
public:
    EventMarkerC() : EventMarker(20) {}

protected:
    void fillRes(const Data::EventMarker& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.eventMarkerC = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder
