#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{
struct SmartHeave
{
    uint32_t validityTime_100us;
    float smartHeave_m;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder
