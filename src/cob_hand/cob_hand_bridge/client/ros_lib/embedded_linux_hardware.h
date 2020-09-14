/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef ROS_EMBEDDED_LINUX_HARDWARE_H_
#define ROS_EMBEDDED_LINUX_HARDWARE_H_

#include <iostream>

#ifdef BUILD_LIBROSSERIALEMBEDDEDLINUX
extern "C" int elCommInit(char *portName, int baud);
extern "C" int elCommRead(int fd);
extern "C" elCommWrite(int fd, uint8_t* data, int length);
#endif

#define DEFAULT_PORT "/dev/ttyAM1"

class EmbeddedLinuxHardware
{
public:
  EmbeddedLinuxHardware(const char *pn, long baud = 57600)
  {
    strncpy(portName, pn, 30);
    baud_ = baud;
  }

  EmbeddedLinuxHardware()
  {
    const char *envPortName = getenv("ROSSERIAL_PORT");
    if (envPortName == NULL)
      strcpy(portName, DEFAULT_PORT);
    else
      strncpy(portName, envPortName, 29);
    portName[29] = '\0'; // in case user gave us too long a port name
    baud_ = 57600;
  }

  void setBaud(long baud)
  {
    this->baud_ = baud;
  }

  int getBaud()
  {
    return baud_;
  }

  void init()
  {
    fd = elCommInit(portName, baud_);
    if (fd < 0)
    {
      std::cout << "Exiting" << std::endl;
      exit(-1);
    }
    std::cout << "EmbeddedHardware.h: opened serial port successfully\n";
    clock_gettime(CLOCK_MONOTONIC, &start);     // record when the program started
  }

  void init(const char *pName)
  {
    fd = elCommInit(pName, baud_);
    if (fd < 0)
    {
      std::cout << "Exiting" << std::endl;
      exit(-1);
    }
    std::cout << "EmbeddedHardware.h: opened comm port successfully\n";
    clock_gettime(CLOCK_MONOTONIC, &start);     // record when the program started
  }

  int read()
  {
    int c = elCommRead(fd);
    return c;
  }

  void write(uint8_t* data, int length)
  {
    elCommWrite(fd, data, length);
  }

  unsigned long time()
  {
    long millis, seconds, nseconds;

    clock_gettime(CLOCK_MONOTONIC, &end);

    seconds  = end.tv_sec  - start.tv_sec;
    nseconds = end.tv_nsec - start.tv_nsec;

    millis = ((seconds) * 1000 + nseconds / 1000000.0) + 0.5;

    return millis;
  }

protected:
  int fd;
  char portName[30];
  long baud_;
  struct timespec start, end;
};

#endif
