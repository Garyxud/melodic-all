#pragma once
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct EventMarker
{
    int32_t validityTime_100us; /* Time tag in steps of 100micros */
    uint8_t event_id;           /* 0 : Event A - 1 : Event B - 2 : Event C */
    uint32_t event_count; /* Number of event received since the last protocol update */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder