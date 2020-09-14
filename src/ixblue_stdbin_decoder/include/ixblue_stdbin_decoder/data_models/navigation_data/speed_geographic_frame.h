#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Speed in geographical frame at primary lever arm */
struct SpeedGeographicFrame
{
    float north_msec; /*! Positive north */
    float east_msec;  /*! Positive east */
    float up_msec;    /*! Positive up */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder