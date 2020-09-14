#pragma once

namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Available only when turret feature is activated */
/*! Position in WGS84 at selected lever arm */
struct VehiclePosition
{
    double latitude_deg;
    double longitude_deg;
    uint8_t altitude_ref; /*! 0 : Geoid (Mean sea level) - 1 : ellipsoid */
    float altitude_m;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder