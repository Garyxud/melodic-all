#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct TurretAngles
{
    int32_t validityTime_100us;          /* Time tag in steps of 100micros */
    float headingbearingdrift_angle_deg; /* Positive clockwise */
    float roll_deg;           /* Positive when right side up - inside [-180°:180°[ */
    float elevationpitch_deg; /* Positive gun up - inside [-90°:90°] */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder