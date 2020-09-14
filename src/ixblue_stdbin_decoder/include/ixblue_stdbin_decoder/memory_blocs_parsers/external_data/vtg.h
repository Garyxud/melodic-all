#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/vtg.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Vtg : public MemoryBlockParser
{
public:
    Vtg(int bit_pose) : MemoryBlockParser(bit_pose, 17) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Vtg res;
        buffer >> res.validityTime_100us >> res.vtg_id >> res.true_course_deg >>
            res.magnetic_course_deg >> res.speed_over_ground_ms;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::Vtg& res, Data::BinaryNav& outBinaryNav) = 0;
};

class Vtg1 : public Vtg
{
public:
    Vtg1() : Vtg(25) {}

protected:
    void fillRes(const Data::Vtg& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.vtg1 = res;
    }
};

class Vtg2 : public Vtg
{
public:
    Vtg2() : Vtg(26) {}

protected:
    void fillRes(const Data::Vtg& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.vtg2 = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder
