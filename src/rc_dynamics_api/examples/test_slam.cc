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

#include <rc_dynamics_api/remote_interface.h>

#include <signal.h>

#ifdef WIN32
#include <winsock2.h>
#undef min
#undef max
#endif

using namespace std;
namespace rcdyn = rc::dynamics;

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
  cout << "\nStarts rc_visard's dynamics and slam modules for a certain time "
          "\nperiod, retrieves the Slam trajectory and simply prints it to std out."
       << "\n\nUsage: \n"
       << arg << " -v <rcVisardIP> [-t <timePeriodSecs>]" << endl;
}

int main(int argc, char* argv[])
{
#ifdef WIN32
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

  // Register signals and signal handler for proper program escape
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);

  /**
   * Parse program options
   */
  string visardIP, networkIface = "", streamName = "";
  bool userSetIp = false;
  unsigned int maxTimeSecs = 5;

  int i = 1;
  while (i < argc)
  {
    std::string p = argv[i++];

    if (p == "-v" && i < argc)
    {
      visardIP = string(argv[i++]);
      userSetIp = true;
    }
    else if (p == "-t" && i < argc)
    {
      maxTimeSecs = (unsigned int)std::max(0, atoi(argv[i++]));
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
  if (!userSetIp)
  {
    cerr << "Please specify rc_visard IP." << endl;
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  /**
   * Instantiate rc::dynamics::RemoteInterface and start streaming
   */
  cout << "connecting rc_visard " << visardIP << "..." << endl;
  auto rcvisardDynamics = rcdyn::RemoteInterface::create(visardIP);

  try
  {
    // start the rc::dynamics module with slam on the rc_visard
    cout << "starting rc_dynamics module with slam on rc_visard..." << endl;
    rcvisardDynamics->startSlam();
  }
  catch (exception& e)
  {
    cout << "ERROR! Could not start rc_dynamics module on rc_visard: " << e.what() << endl;
    return EXIT_FAILURE;
  }

  /**
   * simply wait for defined number of secons
   */
  cout << "running..." << endl;

#ifdef WIN32
  Sleep(1000 * maxTimeSecs);
#else
  usleep(1000 * 1000 * maxTimeSecs);
#endif

  /**
   * Stopping streaming and clean-up
   */
  try
  {
    cout << "stopping rc_dynamics module on rc_visard..." << endl;
    rcvisardDynamics->stop();

    // get the full trajectory and print number of recorded poses
    roboception::msgs::Trajectory traj = rcvisardDynamics->getSlamTrajectory();
    cout << "The full trajectory contains " << traj.poses().size() << " waypoints." << endl;

    // print the last second of the trajectory to cout
    traj = rcvisardDynamics->getSlamTrajectory(rc::TrajectoryTime::RelativeToEnd(1));
    cout << "The last second of the trajectory contains " << traj.poses().size() << " waypoints and looks like:" << endl
         << traj.DebugString() << endl;
  }
  catch (exception& e)
  {
    cout << "ERROR! Could not start rc_dynamics module on rc_visard: " << e.what() << endl;
  }

#ifdef WIN32
  ::WSACleanup();
#endif

  return EXIT_SUCCESS;
}
