#pragma once
#include <array>
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct Usbl
{
    int32_t validityTime_100us;       /* Time tag in steps of 100micros */
    uint8_t usbl_id;                  /* 0 : usbl1 - 1 : usbl2 - 2 : usbl3 */
    std::array<uint8_t, 8> beacon_id; /* 8 ASCII characters */
    double latitude_deg;              /* Positive north - [-90°:90°] */
    double longitude_deg;             /* Positive east - [0°:360°] */
    float altitude_m;                 /* As received from USBL system - cf config */
    float north_stddev_m;
    float east_stddev_m;
    float lat_lon_cov_m2;
    float altitude_stddev_m;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder