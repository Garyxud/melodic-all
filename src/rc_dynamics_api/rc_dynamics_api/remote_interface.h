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

#ifndef RC_DYNAMICS_API_REMOTEINTERFACE_H
#define RC_DYNAMICS_API_REMOTEINTERFACE_H

#include <string>
#include <list>
#include <memory>
#include <iostream>
#include <chrono>

#include "roboception/msgs/frame.pb.h"
#include "roboception/msgs/dynamics.pb.h"
#include "roboception/msgs/imu.pb.h"
#include "roboception/msgs/trajectory.pb.h"

#include "data_receiver.h"
#include "net_utils.h"
#include "trajectory_time.h"

namespace rc
{
namespace dynamics
{
/**
 * Simple remote interface to access the dynamic state estimates
 * of an rc_visard device as data streams.
 *
 * It offers methods to
 *  * interface the rc_dynamics module on the rc_visard device, i.e. starting,
 *      stopping, and checking its state
 *  * manage data streams, i.e. adding, deleting, and checking destinations of
 *      the streams
 *  * an easy-to-use convenience function to directly start listening to a
 *      specific data stream (see createReceiverForStream())
 *
 *  NOTE: For convenience, a RemoteInterface object automatically keeps track
 *      of all data stream destinations requested by itself on the rc_visard
 *      device, and deletes them again when it is going to be destructed.
 *      Therefore, it is highly important that a RemoteInterface is destructed
 *      properly.
 *      In order to do so it, is recommended to wrap method calls of
 *      RemoteInterface objects with try-catch-blocks as they might throw
 *      exceptions and therefore avoid proper destruction of the object.
 */
class RemoteInterface : public std::enable_shared_from_this<RemoteInterface>
{
public:
  using Ptr = std::shared_ptr<RemoteInterface>;

  /// An enum mirroring the state-machine states enum in rc_dynamics/dynamicsRos.h.
  /// The latter is only available on the visard, therefore not included directly.
  struct State
  {
    static const std::string IDLE;                      ///< Not yet started or stopped
    static const std::string RUNNING;                   ///< Stereo INS is running
    static const std::string FATAL;                     ///< An error has occured. May be resolvable by stopping.
    static const std::string STOPPING;                  ///< Intermediate state while transitioning to IDLE (e.g. from RUNNING)
    static const std::string WAITING_FOR_INS;           ///< Waiting for IMU data, will proceed to RUNNING
    static const std::string WAITING_FOR_INS_AND_SLAM;  ///< Waiting for IMU data, will proceed to WAITING_FOR_SLAM
    static const std::string WAITING_FOR_SLAM;   ///< Stereo INS is running, waiting for SLAM data, will proceed to
                                                 ///RUNNING_WITH_SLAM
    static const std::string RUNNING_WITH_SLAM;  ///< Stereo INS and SLAM are running.
    static const std::string UNKNOWN;            ///< State of component is unknown, e.g. not yet reported
  };

  struct ReturnCode
  {
    int value; ///< suceess >= 0, failure < 0
    std::string message;
  };

  /// Thrown if the current_state response of the dynamics service does not correspond to those
  /// in the State struct
  class InvalidState : public std::runtime_error
  {
  public:
    explicit InvalidState(std::string encountered_state)
      : runtime_error("Invalid state encountered: " + encountered_state)
    {
    }
  };

  /// Thrown if a service call is not accepted
  class NotAccepted : public std::runtime_error
  {
  public:
    explicit NotAccepted(std::string service_name) : runtime_error("Service call not accepted: " + service_name)
    {
    }
  };

  /// Thrown if rc_dynamics is requested to receive dynamics data but component is not running
  class DynamicsNotRunning : public std::runtime_error
  {
  public:
    explicit DynamicsNotRunning(std::string state) : runtime_error("No data received: rc_dynamics is not running but in state: " + state)
    {
    }
  };

  /// Thrown if too many streams are running already on rc_visard
  class TooManyStreamDestinations : public std::runtime_error
  {
  public:
    explicit TooManyStreamDestinations(std::string msg) : runtime_error(msg)
    {
    }
  };

  /// Thrown if a REST API call is rejected because of too many requests
  class TooManyRequests : public std::runtime_error
  {
  public:
    explicit TooManyRequests(std::string url) : runtime_error("rc_visard returned http error code 429 (too many requests): " + url)
    {
    }
  };

  /// Thrown if a REST API call is rejected because of 404; i.e. URL not found
  class NotAvailable : public std::runtime_error
  {
  public:
    explicit NotAvailable(std::string url) : runtime_error("Requested resource is not available on rc_visard (returned http error code 404): " + url)
    {
    }
  };

  /**
   * Creates a local instance of rc_visard's remote pose interface
   *
   * @param rc_visard_ip rc_visard's inet address as string, e.g "192.168.0.12"
   * @param requests_timeout timeout in [ms] for doing REST-API calls, which don't have an explicit timeout parameter
   */
  static Ptr create(const std::string& rc_visard_ip, unsigned int requests_timeout = 5000);

  virtual ~RemoteInterface();

  /**
   * Connects with rc_visard and checks the system state of the rc_visard device
   * @return true, if system is ready, false otherwise
   */
  bool checkSystemReady();

  /**
   * Returns the current state of rc_dynamics module
   * @return the current state.
   */
  std::string getDynamicsState();

  /**
   * Returns the current state of rc_slam module
   * @return the current state.
   */
  std::string getSlamState();

  /**
   * Returns the current state of rc_stereo_ins module
   * @return the current state.
   */
  std::string getStereoInsState();

  /**
   * Sets rc_dynamics module to running state.
   * Only start the Stereo INS. To start SLAM use startSlam().
   * To restart use the restart() method.
   * @return the entered state. Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string start();
  /**
   * Sets rc_dynamics module to running state.
   * Also starts up the Stereo INS, if not already running.
   * @return the entered state. Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string startSlam();
  /**
   * Restarts the rc_dynamics module to Stereo INS only mode.
   * Equivalent to stop() and start()
   * @return the entered state. Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string restart();

  /**
   * Restarts the rc_dynamics module to SLAM mode.
   * Equivalent to stop() and startSlam()
   * @return the entered state. Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string restartSlam();

  /**
   * Stops rc_dynamics module. If SLAM is running it will be stopped too.
   * @return the entered state. Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string stop();

  /**
   * Stops only the SLAM module (via the rc_dynamics module).
   * The Stereo INS will keep running.
   * @return the entered state. Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string stopSlam();

  /**
   * Resets the SLAM module
   * The Stereo INS will keep running, if it is.
   * @return the entered state (of the SLAM module). Note that this can be an intermediate state.
   * @throw InvalidState if the entered state does not match the known states in State
   */
  std::string resetSlam();

  /**
   * Saves the SLAM map on the sensor.
   * @param timeout_ms timeout in ms for the call (default 0: no timeout)
   * @return return code indicating success and string message
   */
  ReturnCode saveSlamMap(unsigned int timeout_ms = 0);

  /**
   * Loads the SLAM map on the sensor.
   * @param timeout_ms timeout in ms for the call (default 0: no timeout)
   * @return return code indicating success and string message
   */
  ReturnCode loadSlamMap(unsigned int timeout_ms = 0);

  /**
   * Removes the SLAM map on the sensor.
   * @param timeout_ms timeout in ms for the call (default 0: no timeout)
   * @return return code indicating success and string message
   */
  ReturnCode removeSlamMap(unsigned int timeout_ms = 0);

  /**
   * Returns a list all available streams on rc_visard
   * @return
   */
  std::list<std::string> getAvailableStreams();

  /**
   * Returns the name of the protobuf message class that corresponds to a
   * given data stream and is required for de-serializing the respective
   * messages.
   *
   * @param stream a specific rc_dynamics data stream (e.g. "pose", "pose_rt" or "dynamics")
   * @return the corresponding protobuf message type as string (e.g. "Frame" or "Dynamics")
   */
  std::string getPbMsgTypeOfStream(const std::string& stream);

  /**
   * Returns a list of all destinations registered to the specified
   * rc_dynamics stream.
   * Streams here are represented as their destinations using IP address and
   * port number.
   *
   * @param stream a specific rc_dynamics data stream (e.g. "pose" or "dynamics")
   * @return list of destinations of represented as strings, e.g. "192.168.0.1:30000"
   */
  std::list<std::string> getDestinationsOfStream(const std::string& stream);

  /**
   * Adds a destination to a stream, i.e. request rc_visard to stream data of
   * the specified type to the given destination.
   *
   * @param stream stream type, e.g. "pose", "pose_rt" or "dynamics"
   * @param destination string-represented destination of the data stream, e.g. "192.168.0.1:30000"
   */
  void addDestinationToStream(const std::string& stream, const std::string& destination);

  /**
   * Deletes a destination from a stream, i.e. request rc_visard to stop
   * streaming data of the specified type to the given destination.
   *
   * @param stream stream type, e.g. "pose", "pose_rt" or "dynamics"
   * @param destination string-represented destination of the data stream, e.g. "192.168.0.1:30000"
   */
  void deleteDestinationFromStream(const std::string& stream, const std::string& destination);

  /**
   * Deletes given destinations from a stream, i.e. request rc_visard to stop
   * streaming data of the specified type to the given destinations.
   *
   * @param stream stream type, e.g. "pose", "pose_rt" or "dynamics"
   * @param destinations list string-represented destination of the data stream, e.g. "192.168.0.1:30000"
   */
  void deleteDestinationsFromStream(const std::string& stream, const std::list<std::string>& destinations);

  /**
   * Returns the Slam trajectory from the sensor.
   *
   * Using the start and end arguments only a subsection of the trajectory can
   * be queried. If both are left empy, the full trajectory is returned.
   *
   * @param start specifies the start of the returned trajectory subsection (if empty, the trajectory is returned from
   * its very beginning)
   * @param end specifies the end of the returned trajectory subsection (if empty, the trajectory is included up to its
   * very end)
   */
  roboception::msgs::Trajectory getSlamTrajectory(const TrajectoryTime& start = TrajectoryTime::RelativeToStart(),
                                                  const TrajectoryTime& end = TrajectoryTime::RelativeToEnd(),
                                                  unsigned int timeout_ms = 0);

  /**
   * Returns the transformation from camera to IMU coordinate frame.
   *
   * This is equivalent to the cam2imu_transform in the Dynamics message.
   *
   * @param timeout_ms timeout in ms for the call (default 0: no timeout)
   */
  roboception::msgs::Frame getCam2ImuTransform(unsigned int timeout_ms = 0);

  /**
   * Convenience method that automatically
   *
   *  1) creates a data receiver (including binding socket to a local network interface)
   *  2) adds a destination to the respective stream on rc_visard device
   *  3) waits/checks for the stream being established
   *  4) (removes the destination automatically from rc_visard device if data receiver is no longer used)
   *
   * Stream can only be established successfully if rc_dynamics module is running on
   * rc_visard, see (re)start(_slam) methods.
   *
   *
   * If desired interface for receiving is unspecified (or "") this host's
   * network interfaces are scanned to find a suitable IP address among those.
   * Similar, if port number is unspecified (or 0) it will be assigned
   * arbitrarily as available by network interface layer.
   *
   * @param dest_interface empty or one of this hosts network interfaces, e.g. "eth0"
   * @param dest_port 0 or this hosts port number
   * @return true, if stream could be initialized successfully
   */
  DataReceiver::Ptr createReceiverForStream(const std::string& stream, const std::string& dest_interface = "",
                                            unsigned int dest_port = 0);

protected:
  static std::map<std::string, RemoteInterface::Ptr> remote_interfaces_;

  RemoteInterface(const std::string& rc_visard_ip, unsigned int requests_timeout = 5000);

  void cleanUpRequestedStreams();
  void checkStreamTypeAvailable(const std::string& stream);
  /// Common functionality for start(), startSlam(), stop(), ...
  std::string callDynamicsService(std::string service_name);
  ReturnCode callSlamService(std::string service_name, unsigned int timeout_ms = 0); ///< call slam services which have a return code with value and message
  std::string getState(const std::string& node);

  std::string visard_addrs_;
  bool initialized_;     ///< indicates if remote_interface was initialized properly at least once, see checkSystemReady()
  float visard_version_; ///< rc_visard's firmware version as double, i.e. major.minor, e.g. 1.6
  std::map<std::string, std::list<std::string>> req_streams_;
  std::list<std::string> avail_streams_;
  std::map<std::string, std::string> protobuf_map_;
  std::string base_url_;
  int timeout_curl_;
};
}
}

#endif  // RC_DYNAMICS_API_REMOTEINTERFACE_H
