#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/emlog.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Emlog : public MemoryBlockParser
{
public:
    Emlog(int bit_pos) : MemoryBlockParser(bit_pos, 13) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Emlog res;
        buffer >> res.validityTime_100us >> res.emlog_id >> res.xv1_waterSpeed_ms >>
            res.xv1_speed_stddev_ms;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::Emlog& res, Data::BinaryNav& outBinaryNav) = 0;
};

class Emlog1 : public Emlog
{
public:
    Emlog1() : Emlog(4) {}

private:
    void fillRes(const Data::Emlog& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.emlog1 = res;
    }
};

class Emlog2 : public Emlog
{
public:
    Emlog2() : Emlog(5) {}

private:
    void fillRes(const Data::Emlog& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.emlog2 = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder