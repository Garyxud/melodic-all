#pragma once
#include <array>
#include <inttypes.h>

namespace ixblue_stdbin_decoder
{
namespace Data
{

struct LogBook
{
    int32_t validityTime_100us;          /* Time tag in steps of 100micros */
    uint32_t log_id;                     /* Determined by the application */
    std::array<uint8_t, 32> custom_text; /* String of ASCII characters */
};
} // namespace Data
} // namespace ixblue_stdbin_decoder