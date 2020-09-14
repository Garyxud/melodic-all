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

#include "remote_interface.h"
#include "unexpected_receive_timeout.h"

#include "json.hpp"
#include <cpr/cpr.h>
#include <regex>

using namespace std;
using json = nlohmann::json;

namespace rc
{
namespace dynamics
{
// Definitions of static const members
const std::string RemoteInterface::State::IDLE = "IDLE";
const std::string RemoteInterface::State::RUNNING = "RUNNING";
const std::string RemoteInterface::State::STOPPING = "STOPPING";
const std::string RemoteInterface::State::FATAL = "FATAL";
const std::string RemoteInterface::State::WAITING_FOR_INS = "WAITING_FOR_INS";
const std::string RemoteInterface::State::WAITING_FOR_INS_AND_SLAM = "WAITING_FOR_INS_AND_SLAM";
const std::string RemoteInterface::State::WAITING_FOR_SLAM = "WAITING_FOR_SLAM";
const std::string RemoteInterface::State::RUNNING_WITH_SLAM = "RUNNING_WITH_SLAM";
const std::string RemoteInterface::State::UNKNOWN = "UNKNOWN";

string toString(cpr::Response resp)
{
  stringstream s;
  s << "status code: " << resp.status_code << endl
    << "url: " << resp.url << endl
    << "text: " << resp.text << endl
    << "error: " << resp.error.message;
  return s.str();
}

string toString(list<string> list)
{
  stringstream s;
  s << "[";
  for (auto it = list.begin(); it != list.end();)
  {
    s << *it;
    if (++it != list.end())
    {
      s << ", ";
    }
  }
  s << "]";
  return s.str();
}

void handleCPRResponse(cpr::Response r)
{
  switch (r.status_code) {
    case 200:
      return;
    case 429:
      throw RemoteInterface::TooManyRequests(r.url);
    case 404:
      throw RemoteInterface::NotAvailable(r.url);
    default:
      throw runtime_error(toString(r));
  }
}

namespace {

  vector<int> wait_before_retry = { 50, 200, 500, 1000, 2000};

  // Wrapper around cpr::Get requests which does retries in case of 429 response
  cpr::Response cprGetWithRetry(cpr::Url url, cpr::Timeout timeout) {
    for (int retry : wait_before_retry) {
      auto response = cpr::Get(url, timeout, cpr::Header{ { "accept", "application/json" }});
      if (response.status_code == 429) {
        cout << "WARNING: Got http code 429 (too many requests) on "
             << url << ". Retrying in " << retry << "ms..." << endl;
        usleep(1000 * retry);
        continue;
      }
      return response;
    }
    throw RemoteInterface::TooManyRequests(url);
  }

  // Wrapper around cpr::Put requests which does retries in case of 429 response
  cpr::Response cprPutWithRetry(cpr::Url url, cpr::Timeout timeout, cpr::Body body = cpr::Body{}) {

    // we need different headers if body is empty or not
    cpr::Header header;
    if (body == cpr::Body{}) {
      header = cpr::Header{ { "accept", "application/json" }};
    } else {
      header = cpr::Header{ { "accept", "application/json" }, { "Content-Type", "application/json" }};
    }

    for (int retry : wait_before_retry) {
      auto response = cpr::Put(url, timeout, body, header);
      if (response.status_code==429) {
        cout << "WARNING: Got http code 429 (too many requests) on "
             << url << ". Retrying in " << retry << "ms..." << endl;
        usleep(1000 * retry);
        continue;
      }
      return response;
    }
    throw RemoteInterface::TooManyRequests(url);
  }

  // Wrapper around cpr::Deletee requests which does retries in case of 429 response
  cpr::Response cprDeleteWithRetry(cpr::Url url, cpr::Timeout timeout, cpr::Body body = cpr::Body{}) {

    // we need different headers if body is empty or not
    cpr::Header header;
    if (body == cpr::Body{}) {
      header = cpr::Header{ { "accept", "application/json" }};
    } else {
      header = cpr::Header{ { "accept", "application/json" }, { "Content-Type", "application/json" }};
    }

    for (int retry : wait_before_retry) {
      auto response = cpr::Delete(url, timeout, body, header);
      if (response.status_code==429) {
        cout << "WARNING: Got http code 429 (too many requests) on "
             << url << ". Retrying in " << retry << "ms..." << endl;
        usleep(1000 * retry);
        continue;
      }
      return response;
    }
    throw RemoteInterface::TooManyRequests(url);
  }

}

/**
 * Class for data stream receivers that are created by this
 * remote interface in order to keep track of created streams.
 *
 */
class TrackedDataReceiver : public DataReceiver
{
public:
  static shared_ptr<TrackedDataReceiver> create(const string& ip_address, unsigned int& port, const string& stream,
                                                shared_ptr<RemoteInterface> creator)
  {
    return shared_ptr<TrackedDataReceiver>(new TrackedDataReceiver(ip_address, port, stream, creator));
  }

  virtual ~TrackedDataReceiver()
  {
    try
    {
      creator_->deleteDestinationFromStream(stream_, dest_);
    }
    catch (exception& e)
    {
      cerr << "[TrackedDataReceiver] Could not remove my destination " << dest_ << " for stream type " << stream_
           << " from rc_visard: " << e.what() << endl;
    }
  }

protected:
  TrackedDataReceiver(const string& ip_address, unsigned int& port, const string& stream,
                      shared_ptr<RemoteInterface> creator)
    : DataReceiver(ip_address, port), dest_(ip_address + ":" + to_string(port)), stream_(stream), creator_(creator)
  {
  }

  string dest_, stream_;
  shared_ptr<RemoteInterface> creator_;
};

// map to store already created RemoteInterface objects
map<string, RemoteInterface::Ptr> RemoteInterface::remote_interfaces_ = map<string, RemoteInterface::Ptr>();

RemoteInterface::Ptr RemoteInterface::create(const string& rc_visard_inet_addrs, unsigned int requests_timeout)
{
  // check if interface is already opened
  auto found = RemoteInterface::remote_interfaces_.find(rc_visard_inet_addrs);
  if (found != RemoteInterface::remote_interfaces_.end())
  {
    return found->second;
  }

  // if not, create it
  auto new_remote_interface = Ptr(new RemoteInterface(rc_visard_inet_addrs, requests_timeout));
  RemoteInterface::remote_interfaces_[rc_visard_inet_addrs] = new_remote_interface;

  return new_remote_interface;
}

RemoteInterface::RemoteInterface(const string& rc_visard_ip, unsigned int requests_timeout)
  : visard_addrs_(rc_visard_ip), initialized_(false), visard_version_(0.0),
    base_url_("http://" + visard_addrs_ + "/api/v1"), timeout_curl_(requests_timeout)
{
  req_streams_.clear();
  protobuf_map_.clear();

  // check if given string is a valid IP address
  if (!isValidIPAddress(rc_visard_ip))
  {
    throw invalid_argument("Given IP address is not a valid address: " + rc_visard_ip);
  }
}

RemoteInterface::~RemoteInterface()
{
  try {
    cleanUpRequestedStreams();
  } catch (exception& e) {
    cerr << "[RemoteInterface::~RemoteInterface] Could not clean up all previously requested streams: "
         << e.what() << endl;
  }
  for (const auto& s : req_streams_)
  {
    if (s.second.size() > 0)
    {
      cerr << "[RemoteInterface::~RemoteInterface] Could not stop all previously requested"
              " streams of type "
           << s.first << " on rc_visard. Please check "
                         "device manually"
                         " ("
           << base_url_ << "/datastreams/" << s.first << ")"
                                                        " for not containing any of the following legacy streams and"
                                                        " delete them otherwise, e.g. using the swagger UI ("
           << "http://" + visard_addrs_ + "/api/swagger/)"
           << ": " << toString(s.second) << endl;
    }
  }
}

bool RemoteInterface::checkSystemReady()
{
  initialized_=false;
  visard_version_=0.0;
  req_streams_.clear();
  protobuf_map_.clear();
  avail_streams_.clear();

  cpr::Url url = cpr::Url{ base_url_ + "/system"};
  auto response = cprGetWithRetry(url, cpr::Timeout{ timeout_curl_ });
  if (response.status_code == 502) // bad gateway
  {
    return false;
  }
  handleCPRResponse(response);

  // initial connection to rc_visard to check if system is ready ...
  auto get_system = cprGetWithRetry(cpr::Url{ base_url_ + "/system" },
                             cpr::Timeout{ timeout_curl_ });
  handleCPRResponse(get_system);
  auto j = json::parse(get_system.text);
  if (!j["ready"])
  {
    return false;
  }

  // ... and to get version of rc_visard
  string version = j["firmware"]["active_image"]["image_version"];
  std::smatch match;
  if (std::regex_search(version, match, std::regex("v(\\d+).(\\d+).(\\d+)")))
  {
    visard_version_ = stof(match[0].str().substr(1,3));
  }

  // ...and to get streams
  auto get_streams = cprGetWithRetry(cpr::Url{ base_url_ + "/datastreams" },
                        cpr::Timeout{ timeout_curl_ });
  handleCPRResponse(get_streams);

  // parse text of response into json object
  j = json::parse(get_streams.text);
  for (const auto& stream : j)
  {
    avail_streams_.push_back(stream["name"]);
    protobuf_map_[stream["name"]] = stream["protobuf"];
  }

  // return true if system is ready
  initialized_ = true;
  return true;
}

string RemoteInterface::getState(const std::string& node) {
  cpr::Url url = cpr::Url{ base_url_ + "/nodes/" + node + "/status"};
  auto response = cprGetWithRetry(url, cpr::Timeout{ timeout_curl_ });
  handleCPRResponse(response);
  try
  {
    auto j = json::parse(response.text);
    return j["values"]["state"];
  } catch (std::domain_error &e) {
    // in case "state" field does not exist, return UNKNOWN
    return State::UNKNOWN;
  }
}

string RemoteInterface::getDynamicsState()
{
  return getState("rc_dynamics");
}

string RemoteInterface::getSlamState()
{
  return getState("rc_slam");
}

string RemoteInterface::getStereoInsState()
{
  return getState("rc_stereo_ins");
}

std::string RemoteInterface::callDynamicsService(std::string service_name)
{
  cpr::Url url = cpr::Url{ base_url_ + "/nodes/rc_dynamics/services/" + service_name };
  auto response = cprPutWithRetry(url, cpr::Timeout{ timeout_curl_ });
  handleCPRResponse(response);
  auto j = json::parse(response.text);
  std::string entered_state;
  bool accepted = true;

  try
  {
    const static vector<string> valid_states = {
      State::IDLE,
      State::RUNNING,
      State::STOPPING,
      State::FATAL,
      State::WAITING_FOR_INS,
      State::WAITING_FOR_INS_AND_SLAM,
      State::WAITING_FOR_SLAM,
      State::RUNNING_WITH_SLAM,
      State::UNKNOWN
    };
    entered_state = j["response"]["current_state"].get<std::string>();
    if (std::count(valid_states.begin(), valid_states.end(), entered_state) == 0)
    {
      // mismatch between rc_slam states and states used in this class?
      throw InvalidState(entered_state);
    }

    accepted = j["response"]["accepted"].get<bool>();
  }
  catch (std::logic_error&)
  {
    // Maybe old interface version? If so just return the numeric code
    // as string - it isn't used by the tools using the old interface
    try
    {
      entered_state = std::to_string(j["response"]["enteredState"].get<int>());
    }
    catch (std::logic_error&)
    {
      // Real problem (may even be unrelated to parsing json. Let the user see what the response is.
      cerr << "Logic error when parsing the response of a service call to rc_dynamics!\n";
      cerr << "Service called: " << url << "\n";
      cerr << "Response:"
           << "\n";
      cerr << response.text << "\n";
      throw;
    }
  }

  if (!accepted)
  {
    throw NotAccepted(service_name);
  }

  return entered_state;
}

std::string RemoteInterface::restart()
{
  return callDynamicsService("restart");
}
std::string RemoteInterface::restartSlam()
{
  return callDynamicsService("restart_slam");
}
std::string RemoteInterface::start()
{
  return callDynamicsService("start");
}
std::string RemoteInterface::startSlam()
{
  return callDynamicsService("start_slam");
}
std::string RemoteInterface::stop()
{
  return callDynamicsService("stop");
}
std::string RemoteInterface::stopSlam()
{
  return callDynamicsService("stop_slam");
}

std::string RemoteInterface::resetSlam()
{
  std::string service_name = "reset";
  cpr::Url url = cpr::Url{ base_url_ + "/nodes/rc_slam/services/" + service_name };
  auto response = cprPutWithRetry(url, cpr::Timeout{ timeout_curl_ });
  handleCPRResponse(response);
  auto j = json::parse(response.text);
  std::string entered_state;
  bool accepted = true;

  try
  {
    entered_state = j["response"]["current_state"].get<std::string>();
    std::vector<std::string> valid_states = { "IDLE",       "RUNNING",   "FATAL", "WAITING_FOR_DATA",
                                              "RESTARTING", "RESETTING", "HALTED" };
    if (std::count(valid_states.begin(), valid_states.end(), entered_state) == 0)
    {
      // mismatch between rc_slam states and states used in this class?
      throw InvalidState(entered_state);
    }

    accepted = j["response"]["accepted"].get<bool>();
  }
  catch (std::logic_error& json_exception)
  {
    // Maybe old interface version? If so just return the numeric code
    // as string - it isn't used by the tools using the old interface
    try
    {
      entered_state = std::to_string(j["response"]["enteredState"].get<int>());
    }
    catch (std::logic_error& json_exception)
    {
      // Real problem (may even be unrelated to parsing json. Let the user see what the response is.
      cerr << "Logic error when parsing the response of a service call to rc_dynamics!\n";
      cerr << "Service called: " << url << "\n";
      cerr << "Response:"
           << "\n";
      cerr << response.text << "\n";
      throw;
    }
  }

  if (!accepted)
  {
    throw NotAccepted(service_name);
  }

  return entered_state;
}

RemoteInterface::ReturnCode RemoteInterface::callSlamService(std::string service_name, unsigned int timeout_ms)
{
  cpr::Url url = cpr::Url{ base_url_ + "/nodes/rc_slam/services/" + service_name };
  auto response = cprPutWithRetry(url, cpr::Timeout{ (int32_t)timeout_ms });
  handleCPRResponse(response);
  auto j = json::parse(response.text);

  ReturnCode return_code;

  try
  {
    return_code.value = j["response"]["return_code"]["value"].get<int>();
    return_code.message = j["response"]["return_code"]["message"];
  }
  catch (std::logic_error& json_exception)
  {
    // Real problem (may even be unrelated to parsing json. Let the user see what the response is.
    cerr << "Logic error when parsing the response of a service call to rc_dynamics!\n";
    cerr << "Service called: " << url << "\n";
    cerr << "Response:"
        << "\n";
    cerr << response.text << "\n";
    throw;
  }

  return return_code;
}

RemoteInterface::ReturnCode RemoteInterface::saveSlamMap(unsigned int timeout_ms)
{
  return callSlamService("save_map", timeout_ms);
}
RemoteInterface::ReturnCode RemoteInterface::loadSlamMap(unsigned int timeout_ms)
{
  return callSlamService("load_map", timeout_ms);
}
RemoteInterface::ReturnCode RemoteInterface::removeSlamMap(unsigned int timeout_ms)
{
  return callSlamService("remove_map", timeout_ms);
}

list<string> RemoteInterface::getAvailableStreams()
{
  if (!initialized_ && !checkSystemReady())
  {
    throw std::runtime_error("RemoteInterface not properly initialized or rc_visard is not ready. "
                             "Please initialize with method RemoteInterface::checkSystemReady()!");
  }
  return avail_streams_;
}

string RemoteInterface::getPbMsgTypeOfStream(const string& stream)
{
  checkStreamTypeAvailable(stream);
  return protobuf_map_[stream];
}

list<string> RemoteInterface::getDestinationsOfStream(const string& stream)
{
  checkStreamTypeAvailable(stream);

  list<string> destinations;

  // do get request on respective url (no parameters needed for this simple service call)
  cpr::Url url = cpr::Url{ base_url_ + "/datastreams/" + stream };
  auto get = cprGetWithRetry(url, cpr::Timeout{ timeout_curl_ });
  handleCPRResponse(get);

  // parse result as json
  auto j = json::parse(get.text);
  for (auto dest : j["destinations"])
  {
    destinations.push_back(dest.get<string>());
  }
  return destinations;
}

void RemoteInterface::addDestinationToStream(const string& stream, const string& destination)
{
  checkStreamTypeAvailable(stream);

  // do put request on respective url
  json js_args;
  js_args["destination"] = json::array();
  js_args["destination"].push_back(destination);
  cpr::Url url = cpr::Url{ base_url_ + "/datastreams/" + stream };
  auto put = cprPutWithRetry(url, cpr::Timeout{ timeout_curl_ }, cpr::Body{ js_args.dump() });
  if (put.status_code == 403)
  {
    throw TooManyStreamDestinations(json::parse(put.text)["message"].get<string>());
  }
  handleCPRResponse(put);

  // keep track of added destinations
  req_streams_[stream].push_back(destination);
}

void RemoteInterface::deleteDestinationFromStream(const string& stream, const string& destination)
{
  checkStreamTypeAvailable(stream);

  // do delete request on respective url
  json js_args;
  js_args["destination"] = json::array();
  js_args["destination"].push_back(destination);
  cpr::Url url = cpr::Url{ base_url_ + "/datastreams/" + stream };
  auto del = cprDeleteWithRetry(url, cpr::Timeout{ timeout_curl_ }, cpr::Body{ js_args.dump() });
  handleCPRResponse(del);

  // delete destination also from list of requested streams
  auto& destinations = req_streams_[stream];
  auto found = find(destinations.begin(), destinations.end(), destination);
  if (found != destinations.end())
    destinations.erase(found);
}

void RemoteInterface::deleteDestinationsFromStream(const string& stream, const list<string>& destinations)
{
  checkStreamTypeAvailable(stream);

  // with newer image versions this is the most efficent way, i.e. only one call
  if (visard_version_ >= 1.600001) {

    // do delete request on respective url; list of destinationas are given as body
    json js_destinations = json::array();
    for (const auto& dest: destinations)
    {
      js_destinations.push_back(dest);
    }
    json js_args;
    js_args["destination"] = js_destinations;
    cpr::Url url = cpr::Url{ base_url_ + "/datastreams/" + stream };
    auto del = cprDeleteWithRetry(url, cpr::Timeout{ timeout_curl_ }, cpr::Body{ js_args.dump()});
    handleCPRResponse(del);

  // with older image versions we have to work around and do several calls
  } else {
    for (const auto& dest : destinations)
    {
      // do delete request on respective url
      json js_args;
      js_args["destination"] = json::array();
      js_args["destination"].push_back(dest);
      cpr::Url url = cpr::Url{ base_url_ + "/datastreams/" + stream };
      auto del = cprDeleteWithRetry(url, cpr::Timeout{ timeout_curl_ }, cpr::Body{ js_args.dump() });
      handleCPRResponse(del);
    }
  }

  // delete destination also from list of requested streams
  auto& reqDestinations = req_streams_[stream];
  for (auto& destination : destinations)
  {
    auto found = find(reqDestinations.begin(), reqDestinations.end(), destination);
    if (found != reqDestinations.end())
    {
      reqDestinations.erase(found);
    }
  }
}

namespace
{

// TODO: find an automatic way to parse Messages from Json
// * is possible with protobuf >= 3.0.x
// * https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.util.json_util

roboception::msgs::Trajectory toProtobufTrajectory(const json js)
{
  roboception::msgs::Trajectory pb_traj;

  json::const_iterator js_it;
  if ((js_it = js.find("parent")) != js.end())
  {
    pb_traj.set_parent(js_it.value());
  }
  if ((js_it = js.find("name")) != js.end())
  {
    pb_traj.set_name(js_it.value());
  }
  if ((js_it = js.find("producer")) != js.end())
  {
    pb_traj.set_producer(js_it.value());
  }
  if ((js_it = js.find("timestamp")) != js.end())
  {
    pb_traj.mutable_timestamp()->set_sec(js_it.value()["sec"]);    // TODO: sec
    pb_traj.mutable_timestamp()->set_nsec(js_it.value()["nsec"]);  // TODO: nsec
  }
  for (const auto& js_pose : js["poses"])
  {
    auto pb_pose = pb_traj.add_poses();
    auto pb_time = pb_pose->mutable_timestamp();
    pb_time->set_sec(js_pose["timestamp"]["sec"]);    // TODO: sec
    pb_time->set_nsec(js_pose["timestamp"]["nsec"]);  // TODO: nsec
    auto pb_position = pb_pose->mutable_pose()->mutable_position();
    pb_position->set_x(js_pose["pose"]["position"]["x"]);
    pb_position->set_y(js_pose["pose"]["position"]["y"]);
    pb_position->set_z(js_pose["pose"]["position"]["z"]);
    auto pb_orientation = pb_pose->mutable_pose()->mutable_orientation();
    pb_orientation->set_x(js_pose["pose"]["orientation"]["x"]);
    pb_orientation->set_y(js_pose["pose"]["orientation"]["y"]);
    pb_orientation->set_z(js_pose["pose"]["orientation"]["z"]);
    pb_orientation->set_w(js_pose["pose"]["orientation"]["w"]);
  }
  return pb_traj;
}

roboception::msgs::Frame toProtobufFrame(const json& js, bool producer_optional)
{
  roboception::msgs::Frame pb_frame;

  pb_frame.set_parent(js.at("parent").get<string>());
  pb_frame.set_name(js.at("name").get<string>());

  // if producer is optional don't throw exception if not found
  if (!producer_optional || js.find("producer") != js.end())
  {
    pb_frame.set_producer(js.at("producer").get<string>());
  }

  auto js_pose = js.at("pose");
  auto pb_pose = pb_frame.mutable_pose();
  pb_pose->mutable_timestamp()->set_sec(js_pose.at("timestamp").at("sec"));
  pb_pose->mutable_timestamp()->set_nsec(js_pose.at("timestamp").at("nsec"));

  auto js_pose_pose = js_pose.at("pose");
  auto pb_pose_pose = pb_pose->mutable_pose();
  pb_pose_pose->mutable_position()->set_x(js_pose_pose.at("position").at("x"));
  pb_pose_pose->mutable_position()->set_y(js_pose_pose.at("position").at("y"));
  pb_pose_pose->mutable_position()->set_z(js_pose_pose.at("position").at("z"));
  pb_pose_pose->mutable_orientation()->set_w(js_pose_pose.at("orientation").at("w"));
  pb_pose_pose->mutable_orientation()->set_x(js_pose_pose.at("orientation").at("x"));
  pb_pose_pose->mutable_orientation()->set_y(js_pose_pose.at("orientation").at("y"));
  pb_pose_pose->mutable_orientation()->set_z(js_pose_pose.at("orientation").at("z"));

  return pb_frame;
}

} //anonymous namespace

roboception::msgs::Trajectory RemoteInterface::getSlamTrajectory(const TrajectoryTime& start, const TrajectoryTime& end, unsigned int timeout_ms)
{
  // convert time specification to json obj
  json js_args, js_time, js_start_time, js_end_time;
  js_start_time["sec"] = start.getSec();
  js_start_time["nsec"] = start.getNsec();
  js_end_time["sec"] = end.getSec();
  js_end_time["nsec"] = end.getNsec();
  js_args["args"]["start_time"] = js_start_time;
  js_args["args"]["end_time"] = js_end_time;
  if (start.isRelative())
    js_args["args"]["start_time_relative"] = true;
  if (end.isRelative())
    js_args["args"]["end_time_relative"] = true;

  // put request on slam module to get the trajectory
  cpr::Url url = cpr::Url{ base_url_ + "/nodes/rc_slam/services/get_trajectory" };
  auto get = cprPutWithRetry(url, cpr::Timeout{ (int32_t)timeout_ms }, cpr::Body{ js_args.dump() });
  handleCPRResponse(get);

  auto js = json::parse(get.text)["response"]["trajectory"];
  return toProtobufTrajectory(js);
}

roboception::msgs::Frame RemoteInterface::getCam2ImuTransform(unsigned int timeout_ms) {

  // put request on dynamics module to get the cam2imu transfrom
  cpr::Url url = cpr::Url{ base_url_ + "/nodes/rc_dynamics/services/get_cam2imu_transform" };
  auto get = cprPutWithRetry(url, cpr::Timeout{ (int32_t)timeout_ms });
  handleCPRResponse(get);

  auto js = json::parse(get.text)["response"];
  return toProtobufFrame(js, true);
}

DataReceiver::Ptr RemoteInterface::createReceiverForStream(const string& stream, const string& dest_interface,
                                                           unsigned int dest_port)
{
  checkStreamTypeAvailable(stream);

  // figure out local inet address for streaming
  string dest_address;
  if (!getThisHostsIP(dest_address, visard_addrs_, dest_interface))
  {
    stringstream msg;
    msg << "Could not infer a valid IP address "
           "for this host as the destination of the stream! "
           "Given network interface specification was '"
        << dest_interface << "'.";
    throw invalid_argument(msg.str());
  }

  // create data receiver with port as specified
  DataReceiver::Ptr receiver = TrackedDataReceiver::create(dest_address, dest_port, stream, shared_from_this());

  // do REST-API call requesting a UDP stream from rc_visard device
  string destination = dest_address + ":" + to_string(dest_port);
  addDestinationToStream(stream, destination);

  // waiting for first message; we set a long timeout for receiving data
  unsigned int initial_timeOut = 5000;
  receiver->setTimeout(initial_timeOut);
  if (!receiver->receive(protobuf_map_[stream]))
  {
    // we did not receive any message; check why, e.g. dynamics not in correct state?
    string current_state = getDynamicsState();
    std::vector<std::string> valid_states = { "RUNNING",  "RUNNING_WITH_SLAM" };
    if (std::count(valid_states.begin(), valid_states.end(), current_state) == 0)
    {
      throw DynamicsNotRunning(current_state);
    }

    // in other cases we cannot tell, what's the reason
    throw UnexpectedReceiveTimeout(initial_timeOut);
  }

  // stream established, prepare everything for normal pose receiving
  receiver->setTimeout(100);
  return receiver;
}

void RemoteInterface::cleanUpRequestedStreams()
{
  // for each stream type stop all previously requested streams
  for (auto const& s : req_streams_)
  {
    if (!s.second.empty())
    {
      deleteDestinationsFromStream(s.first, s.second);
    }
  }
}

void RemoteInterface::checkStreamTypeAvailable(const string& stream)
{
  if (!initialized_ && !checkSystemReady())
  {
    throw std::runtime_error("RemoteInterface not properly initialized or rc_visard is not ready. "
                             "Please initialize with method RemoteInterface::checkSystemReady()!");
  }
  auto found = find(avail_streams_.begin(), avail_streams_.end(), stream);
  if (found == avail_streams_.end())
  {
    stringstream msg;
    msg << "Stream of type '" << stream << "' is not available on rc_visard " << visard_addrs_;
    throw invalid_argument(msg.str());
  }
}
}
}
