#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct INSAlgorithmStatus
{
    uint32_t status1;
    uint32_t status2;
    uint32_t status3;
    uint32_t status4;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder