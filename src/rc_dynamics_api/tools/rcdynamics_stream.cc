/*
 * This file is part of the rc_dynamics_api package.
 *
 * Copyright (c) 2017 Roboception GmbH
 * All rights reserved
 *
 * Author: Christian Emmerich
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <signal.h>
#include <chrono>
#include <iomanip>

#include "rc_dynamics_api/remote_interface.h"
#include "csv_printing.h"

#ifdef WIN32
#include <winsock2.h>
#undef max
#undef min
#endif

using namespace std;
using namespace rc::dynamics;

/**
 * catching signals for proper program escape
 */
static bool caught_signal = false;
void signal_callback_handler(int signum)
{
  printf("Caught signal %d, stopping program!\n", signum);
  caught_signal = true;
}

/**
 * Print usage of example including command line args
 */
void printUsage(char* arg)
{
  cout << "\nLists available rcdynamics data streams of the specified rc_visard IP, "
          "\nor requests a data stream and either prints received messages or records "
          "\nthem as csv-file, see -o option."
       << "\n\nUsage: \n"
       << arg << " -v <rcVisardIP> -l | -s <stream> [-a] [-i <networkInterface>]"
                 " [-n <maxNumData>][-t <maxRecTimeSecs>][-o <output_file>]"
       << endl;
}

int main(int argc, char* argv[])
{
#ifdef WIN32
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

  // Register signals and signal handler
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);

  /**
   * Parse program options (e.g. IP )
   */
  string out_file_name, visard_ip, network_iface = "", stream_name;
  unsigned int max_num_recording = 50, max_secs_recording = 5;
  bool user_autostart = false;
  bool user_set_out_file = false;
  bool user_set_max_num_msgs = false;
  bool user_set_max_recording_time = false;
  bool user_set_ip = false;
  bool user_set_stream_type = false;
  bool only_list_streams = false;

  int i = 1;
  while (i < argc)
  {
    std::string p = argv[i++];

    if (p == "-l")
    {
      only_list_streams = true;
    }
    else if (p == "-s" && i < argc)
    {
      stream_name = string(argv[i++]);
      user_set_stream_type = true;
    }
    else if (p == "-a")
    {
      user_autostart = true;
    }
    else if (p == "-i" && i < argc)
    {
      network_iface = string(argv[i++]);
    }
    else if (p == "-v" && i < argc)
    {
      visard_ip = string(argv[i++]);
      user_set_ip = true;
    }
    else if (p == "-n" && i < argc)
    {
      max_num_recording = (unsigned int)std::max(0, atoi(argv[i++]));
      user_set_max_num_msgs = true;
    }
    else if (p == "-t" && i < argc)
    {
      max_secs_recording = (unsigned int)std::max(0, atoi(argv[i++]));
      user_set_max_recording_time = true;
    }
    else if (p == "-o" && i < argc)
    {
      out_file_name = string(argv[i++]);
      user_set_out_file = true;
    }
    else if (p == "-h")
    {
      printUsage(argv[0]);
      return EXIT_SUCCESS;
    }
    else
    {
      printUsage(argv[0]);
      return EXIT_FAILURE;
    }
  }

  if (!user_set_ip)
  {
    cerr << "Please specify rc_visard IP." << endl;
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  if (!user_set_stream_type && !only_list_streams)
  {
    cerr << "Please specify stream type." << endl;
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  if (!user_set_max_num_msgs && !user_set_max_recording_time)
  {
    user_set_max_num_msgs = true;
  }

  /**
   * open file for recording if required
   */
  ofstream output_file;
  if (user_set_out_file)
  {
    output_file.open(out_file_name);
    if (!output_file.is_open())
    {
      cerr << "Could not open file '" << out_file_name << "' for writing!" << endl;
      return EXIT_FAILURE;
    }
  }

  /**
   * Instantiate and connect RemoteInterface
   */
  cout << "connecting to rc_visard " << visard_ip << "..." << endl;
  auto rc_dynamics = RemoteInterface::create(visard_ip);
  try {
    while (!caught_signal && !rc_dynamics->checkSystemReady())
    {
      cout << "... system not yet ready. Trying again." << endl;
      usleep(1000*500);
    }
    cout << "... connected!" << endl;
  } catch (exception &e) {
    cout << "ERROR! Could not connect to rc_dynamics module on rc_visard: " << e.what() << endl;
    return EXIT_FAILURE;
  }

  /* Only list available streams of device and exit */
  if (only_list_streams)
  {
    auto streams = rc_dynamics->getAvailableStreams();
    string first_column = "Available streams:";
    size_t first_column_width = first_column.length();
    for (auto&& s : streams)
      if (s.length() > first_column_width)
        first_column_width = s.length();
    first_column_width += 5;
    cout << left << setw(first_column_width) << first_column << "Protobuf message types:" << endl;
    for (auto&& s : streams)
      cout << left << setw(first_column_width) << s << rc_dynamics->getPbMsgTypeOfStream(s) << endl;

    cout << endl << "rc_dynamics is in state: " << rc_dynamics->getDynamicsState();
    cout << endl << "rc_slam is in state: " << rc_dynamics->getSlamState();
    cout << endl << "rc_stereo_ins is in state: " << rc_dynamics->getStereoInsState() << endl;

    try {
      auto cam2imu = rc_dynamics->getCam2ImuTransform();
      cout << endl << "cam2imu transformation: " << endl << cam2imu.DebugString();
    } catch (RemoteInterface::NotAvailable& e) {
      cout << endl << "WARN: Could not retrieve cam2imu transformation from rc_visard. Feature is not available in that image version." << endl;
    } catch (std::exception &e) {
      cout << endl << "ERROR: Could not retrieve cam2imu transformation from rc_visard: " << e.what() << endl;
    }
    cout << endl;
    return EXIT_SUCCESS;
  }

  /* For all streams except 'imu' the rc_dynamcis node has to be started */
  if (user_autostart && stream_name != "imu")
  {
    try
    {
      cout << "starting SLAM on rc_visard..." << endl;
      rc_dynamics->startSlam();
    }
    catch (exception&)
    {
      try
      {
        // start the rc::dynamics module on the rc_visard
        cout << "SLAM not available!" << endl;
        cout << "starting stereo INS on rc_visard..." << endl;
        rc_dynamics->start();
      }
      catch (exception& e)
      {
        cout << "ERROR! Could not start rc_dynamics module on rc_visard: " << e.what() << endl;
        return EXIT_FAILURE;
      }
    }
  }

  /**
   * Request a data stream and start receiving as well as processing the data
   */
  unsigned int cnt_msgs = 0;
  try
  {
    cout << "Initializing " << stream_name << " data stream..." << endl;
    auto receiver = rc_dynamics->createReceiverForStream(stream_name, network_iface);

    unsigned int timeout_millis = 100;
    receiver->setTimeout(timeout_millis);
    cout << "Listening for " << stream_name << " messages..." << endl;

    chrono::time_point<chrono::system_clock> start = chrono::system_clock::now();
    chrono::duration<double> elapsed_secs(0);
    while (!caught_signal && (!user_set_max_num_msgs || cnt_msgs < max_num_recording) &&
           (!user_set_max_recording_time || elapsed_secs.count() < max_secs_recording))
    {
      auto msg = receiver->receive(rc_dynamics->getPbMsgTypeOfStream(stream_name));
      if (msg)
      {
        if (output_file.is_open())
        {
          if (cnt_msgs == 0)
          {
            csv::Header h;
            output_file << (h << *msg) << endl;
          }
          csv::Line l;
          output_file << (l << *msg) << endl;
        }
        else
        {
          cout << "received " << stream_name << " msg:" << endl << msg->DebugString() << endl;
        }
        ++cnt_msgs;
      }
      else
      {
        cerr << "did not receive any data during last " << timeout_millis << " ms." << endl;
      }
      elapsed_secs = chrono::system_clock::now() - start;
    }
  }
  catch (exception& e)
  {
    cout << "Caught exception during streaming, stopping: " << e.what() << endl;
  }

  /**
   * Stopping streaming and clean-up
   * 'imu' stream works regardless if the rc_dynamics module is running, so no need to stop it
   */
  if (user_autostart && stream_name != "imu")
  {
    try
    {
      cout << "stopping rc_dynamics module on rc_visard..." << endl;
      rc_dynamics->stop();
    }
    catch (exception& e)
    {
      cout << "Caught exception: " << e.what() << endl;
    }
  }

  if (output_file.is_open())
  {
    output_file.close();
    cout << "Recorded " << cnt_msgs << " " << stream_name << " messages to '" << out_file_name << "'." << endl;
  }
  else
  {
    cout << "Received  " << cnt_msgs << " " << stream_name << " messages." << endl;
  }

#ifdef WIN32
  ::WSACleanup();
#endif

  return EXIT_SUCCESS;
}
