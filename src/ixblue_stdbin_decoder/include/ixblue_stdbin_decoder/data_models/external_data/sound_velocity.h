#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

/* External sound velocity received from sound velocity probe */
struct SoundVelocity
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    float ext_speedofsound_ms;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder