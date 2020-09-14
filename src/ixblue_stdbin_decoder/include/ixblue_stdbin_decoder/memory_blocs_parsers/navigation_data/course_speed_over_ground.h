#pragma once

#include "ixblue_stdbin_decoder/data_models/navigation_data/course_speed_over_ground.h"
#include "ixblue_stdbin_decoder/memory_block_parser.h"

namespace ixblue_stdbin_decoder
{
namespace Parser
{
class CourseSpeedoverGround : public MemoryBlockParser
{
public:
    CourseSpeedoverGround() : MemoryBlockParser(24, 8) {}
    void parse(boost::asio::const_buffer& buffer,
               Data::BinaryNav& outBinaryNav) override
    {
        Data::CourseSpeedoverGround res;
        buffer >> res.course_over_ground >> res.speed_over_ground;
        outBinaryNav.courseSpeedoverGround = res;
    }
};
} // namespace Parser
} // namespace ixblue_stdbin_decoder