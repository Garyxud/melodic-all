#pragma once

namespace ixblue_stdbin_decoder
{
namespace Data
{
struct RealTimeHeaveSurgeSway
{
    float rt_heave_withoutBdL; /*! Meters - positive UP in horizontal vehicle frame */
    float rt_heave_atBdL;      /*! Meters - positive UP in horizontal vehicle frame */
    float rt_surge_atBdL; /*! Meters - positive FORWARD in horizontal vehicle frame */
    float rt_sway_atBdL;  /*! Meters - positive PORT SIDE in horizontal vehicle frame */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder