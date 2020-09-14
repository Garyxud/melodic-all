#pragma once

namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Available only when turret feature is activated */
struct VehicleAttitudeHeadingDeviation
{
    float heading_stddev_deg; /*! Inside 0°-360° */
    float roll_stddev_deg;    /*! Inside 0°-360° */
    float pitch_stddev_deg;   /*! Inside 0°-360° */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder