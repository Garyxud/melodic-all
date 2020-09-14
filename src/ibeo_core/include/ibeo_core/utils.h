/*
 * MIT License
 *
 * Copyright (c) 2018 AutonomouStuff, LLC
 *
 */

#ifndef IBEO_CORE_UTILS_H
#define IBEO_CORE_UTILS_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <vector>

#include <network_interface/network_utils.h>

using namespace AS::Network;  // NOLINT

namespace AS
{
namespace Drivers
{
namespace Ibeo
{
const size_t MAGIC_WORD = 0xAFFEC0C2;
typedef uint64_t NTPTime;

template<typename T>
void parse_tuple(uint8_t *in, T *out1, T *out2, ByteOrder bo)
{
  size_t bytes = sizeof(T);

  if (bo == LE)
  {
    *out1 = read_le<T>(in, bytes, 0);
    *out2 = read_le<T>(in, bytes, bytes);
  }
  else if (bo == BE)
  {
    *out1 = read_be<T>(in, bytes, 0);
    *out2 = read_be<T>(in, bytes, bytes);
  }
};

inline NTPTime unix_time_to_ntp(struct tm *tm, struct timeval *tv)
{
  NTPTime ret_val;

  uint64_t ntp_sec = tm->tm_year * 31536000;  // Add years since 1900 (in seconds)
  ntp_sec += tm->tm_yday * 86400;                  // Add days since Jan 1st (in seconds)
  ntp_sec += tm->tm_hour * 3600;                   // Add hours since midnight (in seconds)
  ntp_sec += tm->tm_min * 60;                      // Add minutes after the hour (in seconds)
  ntp_sec += tm->tm_sec;                           // Add seconds after the minute

  ret_val = (ntp_sec << 32);

  uint64_t ntp_frac = tv->tv_usec;

  ret_val |= ntp_frac;

  return ret_val;
}

inline double ticks_to_angle(int16_t angle_ticks, uint16_t angle_ticks_per_rotation)
{
  return (2.0 * M_PI * static_cast<double>(angle_ticks) / static_cast<double>(angle_ticks_per_rotation));
};
}  // namespace Ibeo
}  // namespace Drivers
}  // namespace AS

#endif  // IBEO_CORE_UTILS_H
