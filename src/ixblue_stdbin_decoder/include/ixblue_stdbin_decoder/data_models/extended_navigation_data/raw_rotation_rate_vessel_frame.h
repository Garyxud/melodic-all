#pragma once

namespace ixblue_stdbin_decoder
{
namespace Data
{

/*! Raw Rotation Rate not compensated from earth rotation */
struct RawRotationRateVesselFrame
{
    float xv1_degsec;
    float xv2_degsec;
    float xv3_degsec;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder