#pragma once
#include <array>
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct Lbl
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t rfu;                /* Reserved for futur used */
    std::array<uint8_t, 8> beacon_id;
    double beacon_latitude_deg;  /* Positive north - [-90°:90°] */
    double beacon_longitude_deg; /* Positive east - [0°:360°] */
    float beacon_altitude_m;     /* Positive up */
    float range_m;               /* Positive */
    float range_stddev_m;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder