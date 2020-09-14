#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct Utc
{
    uint32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t source;              /* INS can have 2 sources for UTC data */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder