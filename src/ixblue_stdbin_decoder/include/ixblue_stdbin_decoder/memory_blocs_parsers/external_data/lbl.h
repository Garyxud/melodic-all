#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/lbl.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Lbl : public MemoryBlockParser
{
public:
    Lbl(int bit_pose) : MemoryBlockParser(bit_pose, 41) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Lbl res;
        buffer >> res.validityTime_100us >> res.rfu >> res.beacon_id >>
            res.beacon_latitude_deg >> res.beacon_longitude_deg >>
            res.beacon_altitude_m >> res.range_m >> res.range_stddev_m;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::Lbl& res, Data::BinaryNav& outBinaryNav) = 0;
};

class Lbl1 : public Lbl
{
public:
    Lbl1() : Lbl(14) {}

protected:
    void fillRes(const Data::Lbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.lbl1 = res;
    }
};

class Lbl2 : public Lbl
{
public:
    Lbl2() : Lbl(15) {}

protected:
    void fillRes(const Data::Lbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.lbl2 = res;
    }
};

class Lbl3 : public Lbl
{
public:
    Lbl3() : Lbl(16) {}

protected:
    void fillRes(const Data::Lbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.lbl3 = res;
    }
};

class Lbl4 : public Lbl
{
public:
    Lbl4() : Lbl(17) {}

protected:
    void fillRes(const Data::Lbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.lbl4 = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder
