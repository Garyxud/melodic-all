#pragma once

namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Available only when turret feature is activated */
struct VehicleAttitudeHeading
{
    float heading_deg; /*! Inside 0°/360° */
    float roll_deg;    /*! Inside -180°/180° - Positive when right side up */
    float pitch_deg;   /*! Inside -90°/90° - Positive when front down */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder