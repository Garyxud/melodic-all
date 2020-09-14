#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{

struct CurrentGeographicFrameDeviation
{
    float north_stddev_msec; /*! Positive north */
    float east_stddev_msec;  /*! Positive east */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder