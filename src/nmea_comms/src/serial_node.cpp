/**
Software License Agreement (BSD)

\file      serial_node.cpp
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

#include "ros/ros.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "nmea_comms/rx.h"


void manage_connection(const ros::TimerEvent& event, ros::NodeHandle& n, std::string port, int32_t baud, std::string frame_id)
{
  if (rx_prune_threads() > 0)
  {
    // Serial thread already active. Nothing to do here.
    return;
  }

  // Only output error messages when things were previously peachy.
  static int previous_success = 1;

  while (ros::ok())
  {
    ROS_DEBUG_STREAM("Opening serial port: " << port);
    int serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

    if (serial_fd == -1)
    {
      if (previous_success)
      {
        ROS_ERROR_STREAM("Could not open " << port << ".");
      }
      goto retry_connection;
    }

    if (!isatty(serial_fd))
    {
      if (previous_success)
      {
        ROS_ERROR_STREAM("File " << port << " is not a tty.");
      }
      goto close_serial;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    options.c_cflag = 0;
    options.c_cflag |= CS8;

    options.c_cflag |= (CLOCAL | CREAD);

    ROS_DEBUG_STREAM("Setting baud rate: " << baud);
    switch (baud)
    {
    case 9600:
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      break;
    case 19200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;
    case 38400:
      cfsetispeed(&options, B38400);
      cfsetospeed(&options, B38400);
      break;
    case 57600:
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      break;
    case 115200:
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      break;
    default:
      ROS_FATAL_STREAM("Unsupported baud rate: " << baud);
      ros::shutdown();
    }
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;
    tcsetattr(serial_fd, TCSAFLUSH, &options);

    // Successful connection setup; kick off servicing thread.
    ROS_INFO_STREAM("Successfully connected on " << port << ".");
    previous_success = 1;
    rx_thread_start(n, serial_fd, frame_id, 1000000000 / baud * 10);
    break;

close_serial:
    close(serial_fd);

retry_connection:
    if (previous_success)
    {
      ROS_ERROR("Retrying connection every 1 second.");
      previous_success = 0;
    }

    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_serial_node");
  ros::NodeHandle n_local("~");
  ros::NodeHandle n;

  std::string port;
  int32_t baud;
  n_local.param<std::string>("port", port, "/dev/ttyUSB0");
  n_local.param("baud", baud, 115200);

  std::string frame_id;
  n_local.param<std::string>("frame_id", frame_id, "navsat");

  ros::Timer timer = n.createTimer(ros::Duration(1.0),
                                   boost::bind(manage_connection, _1, n, port, baud, frame_id));
  ros::spin();

  rx_stop_all();

  return 0;
}
