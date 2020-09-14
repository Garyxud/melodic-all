#pragma once
namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Rotation compensated from earth rotation */
struct RotationRateVesselFrame
{
    float xv1_degsec; /*! Deg/s - Positive when port going up */
    float xv2_degsec; /*! Deg/s - Positive bow port going down */
    float xv3_degsec; /*! Deg/s - Positive counter clock wise */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder