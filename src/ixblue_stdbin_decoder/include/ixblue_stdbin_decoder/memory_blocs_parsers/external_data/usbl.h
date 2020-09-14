#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/usbl.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Usbl : public MemoryBlockParser
{
public:
    Usbl(int bit_pose) : MemoryBlockParser(bit_pose, 49) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Usbl res;
        buffer >> res.validityTime_100us >> res.usbl_id >> res.beacon_id >>
            res.latitude_deg >> res.longitude_deg >> res.altitude_m >>
            res.north_stddev_m >> res.east_stddev_m >> res.lat_lon_cov_m2 >>
            res.altitude_stddev_m;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::Usbl& res, Data::BinaryNav& outBinaryNav) = 0;
};

class Usbl1 : public Usbl
{
public:
    Usbl1() : Usbl(6) {}

protected:
    void fillRes(const Data::Usbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.usbl1 = res;
    }
};

class Usbl2 : public Usbl
{
public:
    Usbl2() : Usbl(7) {}

protected:
    void fillRes(const Data::Usbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.usbl2 = res;
    }
};

class Usbl3 : public Usbl
{
public:
    Usbl3() : Usbl(8) {}

protected:
    void fillRes(const Data::Usbl& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.usbl3 = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder
