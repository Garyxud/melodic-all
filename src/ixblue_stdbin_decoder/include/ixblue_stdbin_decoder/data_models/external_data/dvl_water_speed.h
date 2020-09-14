#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct DvlWaterSpeed
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t dvl_id;             /* 0 : dvl1 - 1 : dvl2 */
    float xv1_waterspeed_ms;    /* Longitudinal ground speed */
    float xv2_waterspeed_ms;    /* Transverse ground speed */
    float xv3_waterspeed_ms;    /* Vertical ground speed */
    float dvl_speedofsound_ms;
    float xv1_stddev_ms;
    float xv2_stddev_ms;
    float xv3_stddev_ms;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder