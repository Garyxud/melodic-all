#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Speed in vessel frame at primary lever arm */
struct SpeedVesselFrame
{
    float xv1_msec; /*! Positive along xv1 */
    float xv2_msec; /*! Positive along xv2 */
    float xv3_msec; /*! Positive along xv3 */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder