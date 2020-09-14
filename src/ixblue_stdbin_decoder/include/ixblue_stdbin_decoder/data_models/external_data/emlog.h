#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct Emlog
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t emlog_id;           /* 0 : emlog1 - 1 : emlog2 */
    float xv1_waterSpeed_ms;    /* Longitudinal water speed - positive forward */
    float xv1_speed_stddev_ms;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder