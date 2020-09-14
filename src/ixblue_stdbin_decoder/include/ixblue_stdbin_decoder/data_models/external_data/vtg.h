#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct Vtg
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t vtg_id;             /* 0 : VTG1 - 1 : VTG2 */
    float true_course_deg;
    float magnetic_course_deg;
    float speed_over_ground_ms;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder