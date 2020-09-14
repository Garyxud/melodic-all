/**
Software License Agreement (BSD)

\file      tx.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "nmea_comms/rx.h"
#include "nmea_comms/checksum.h"

#include <stdio.h>
#include <poll.h>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"


void tx_msg_callback(const nmea_msgs::SentenceConstPtr sentence_msg_ptr, int fd)
{
  static int consecutive_errors = 0;

  char buffer[256];
  int buffer_length = snprintf(buffer, 256, "%s\r\n", sentence_msg_ptr->sentence.c_str());

  // No guarantee that write() will write everything, so we use poll() to block
  // on the availability of the fd for writing until the whole message has been
  // written out.
  const char* buffer_write = buffer;
  struct pollfd pollfds[] = { { fd, POLLOUT, 0 } };
  while (ros::ok())
  {
    int retval = poll(pollfds, 1, 1000);

    if (pollfds[0].revents & POLLHUP)
    {
      ROS_INFO("Device hangup occurred on attempted write.");
      return;
      //ros::shutdown();
    }

    if (pollfds[0].revents & POLLERR)
    {
      ROS_FATAL("Killing node due to device error.");
      ros::shutdown();
    }

    retval = write(fd, buffer_write, buffer_length - (buffer_write - buffer));
    if (retval > 0)
    {
      buffer_write += retval;
    }
    else
    {
      ROS_WARN("Device write error; abandoning message (%s).",
               sentence_msg_ptr->sentence.c_str());
      if (++consecutive_errors >= 10)
      {
        ROS_FATAL("Killing node due to %d consecutive write errors.", consecutive_errors);
        ros::shutdown();
      }
      break;
    }
    if (buffer_write - buffer >= buffer_length)
    {
      consecutive_errors = 0;
      break;
    }
  }
}
