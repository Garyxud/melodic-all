#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{
// TODO : For all status data, we will need to create a status structure which describe
// all bits in the status, and allow user to query a textual representation of a status.
struct INSSystemStatus
{
    uint32_t status1;
    uint32_t status2;
    uint32_t status3;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder
