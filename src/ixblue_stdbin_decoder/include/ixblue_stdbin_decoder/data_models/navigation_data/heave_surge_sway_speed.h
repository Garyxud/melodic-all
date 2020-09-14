#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{
struct HeaveSurgeSwaySpeed
{
    float realtime_heave_speed; /*! meter/sec - positive up in horizontal vehicle frame */
    float surge_speed; /*! meter/sec - positive forward in horizontal vehicle frame */
    float sway_speed;  /*! meter/sec - positive port side in horizontal vehicle frame */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder