#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{
struct HeadingRollPitchRate
{
    float heading_rate; /*! Deg/s - Positive when heading increase */
    float roll_rate;    /*! Deg/s - Positive when port going up */
    float pitch_rate;   /*! Deg/s - Positive when bow going down */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder
