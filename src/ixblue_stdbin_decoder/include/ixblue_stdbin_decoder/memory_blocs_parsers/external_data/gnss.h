#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/gnss.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class Gnss : public MemoryBlockParser
{
public:
    Gnss(int bit_pose) : MemoryBlockParser(bit_pose, 46) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::Gnss res;
        buffer >> res.validityTime_100us >> res.gnss_id >> res.gnss_quality >>
            res.latitude_deg >> res.longitude_deg >> res.altitude_m >>
            res.latitude_stddev_m >> res.longitude_stddev_m >> res.altitude_stddev_m >>
            res.lat_lon_stddev_m2 >> res.geoidal_separation_m;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::Gnss& res, Data::BinaryNav& outBinaryNav) = 0;
};

class Gnss1 : public Gnss
{
public:
    Gnss1() : Gnss(1) {}

protected:
    void fillRes(const Data::Gnss& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.gnss1 = res;
    }
};

class Gnss2 : public Gnss
{
public:
    Gnss2() : Gnss(2) {}

protected:
    void fillRes(const Data::Gnss& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.gnss2 = res;
    }
};

class GnssManual : public Gnss
{
public:
    GnssManual() : Gnss(3) {}

protected:
    void fillRes(const Data::Gnss& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.gnssManual = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder