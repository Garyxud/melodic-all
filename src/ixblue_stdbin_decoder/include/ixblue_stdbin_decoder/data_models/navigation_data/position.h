#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Position in WGS84 at selected lever arm */
struct Position
{
    double latitude_deg;  /* Positive north - [-90°:90°] */
    double longitude_deg; /* Increasing toward east - [0°:360°] */
    uint8_t altitude_ref; /*! 0 : Geoid (Mean sea level) - 1 : ellipsoid */
    float altitude_m;     /* Positive up */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder