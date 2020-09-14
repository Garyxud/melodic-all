#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct Gnss
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t gnss_id;            /* 0 : gnss1 - 1 : gnss2 - 2 : manual gnss */
    uint8_t gnss_quality;       /* TODO : decode Natural, differential, military ... */
    double latitude_deg;        /* Positive north - [-90°:90°] */
    double longitude_deg;       /* Increasing toward east - [0°:360°] */
    float altitude_m;           /* Positive up - in geoid ref */
    float latitude_stddev_m;
    float longitude_stddev_m;
    float altitude_stddev_m;
    float lat_lon_stddev_m2; /* 0 for manual GNSS */
    float geoidal_separation_m;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder