#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include "helpers.h"

double gettime()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (uint64_t)(curtime.tv_sec) + (uint64_t)(curtime.tv_nsec) * 1e-9;  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (uint64_t)(timeofday.tv_sec) + (uint64_t)(timeofday.tv_usec) * 1e-6;
#endif
}
