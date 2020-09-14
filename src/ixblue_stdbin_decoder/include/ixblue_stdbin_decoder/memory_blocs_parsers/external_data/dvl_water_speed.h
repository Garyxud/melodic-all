#pragma once

#include "ixblue_stdbin_decoder/data_models/external_data/dvl_water_speed.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class DvlWaterSpeed : public MemoryBlockParser
{
public:
    DvlWaterSpeed(int bit_pose) : MemoryBlockParser(bit_pose, 33) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::DvlWaterSpeed res;
        buffer >> res.validityTime_100us >> res.dvl_id >> res.xv1_waterspeed_ms >>
            res.xv2_waterspeed_ms >> res.xv3_waterspeed_ms >> res.dvl_speedofsound_ms >>
            res.xv1_stddev_ms >> res.xv2_stddev_ms >> res.xv3_stddev_ms;
        fillRes(res, outBinaryNav);
    }
    virtual void fillRes(const Data::DvlWaterSpeed& res,
                         Data::BinaryNav& outBinaryNav) = 0;
};

class DvlWaterSpeed1 : public DvlWaterSpeed
{
public:
    DvlWaterSpeed1() : DvlWaterSpeed(11) {}

protected:
    void fillRes(const Data::DvlWaterSpeed& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.dvlWaterSpeed1 = res;
    }
};

class DvlWaterSpeed2 : public DvlWaterSpeed
{
public:
    DvlWaterSpeed2() : DvlWaterSpeed(22) {}

protected:
    void fillRes(const Data::DvlWaterSpeed& res, Data::BinaryNav& outBinaryNav)
    {
        outBinaryNav.dvlWaterSpeed2 = res;
    }
};

} // namespace Parser
} // namespace ixblue_stdbin_decoder
