#include <tf2_server/tf2_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

namespace tf2_server
{

void warnLeadingSlash(const std::string& s)
{
  ROS_WARN_STREAM("Found initial slash in " << s);
}

std::string stripLeadingSlash(const std::string &s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn) {
      warnLeadingSlash(s);
    }
    return s.substr(1);
  }

  return s;
}

TF2Server::TF2Server(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh(nh), pnh(pnh)
{
  double buffer_size;
  this->nh.param("buffer_size", buffer_size, 120.0);

  bool publish_frame_service;
  this->nh.param("publish_frame_service", publish_frame_service, false);

  // Legacy behavior re: #209
  bool use_node_namespace;
  this->nh.param("use_node_namespace", use_node_namespace, false);
  std::string node_name;
  if (use_node_namespace)
  {
    node_name = this->pnh.getNamespace();
  }
  else
  {
    node_name = "tf2_buffer_server";
  }

  this->pnh.setParam("supports_transform_streams", true);

  double duration;
  this->pnh.param("initial_streams_wait_time", duration, 1.0);
  this->initialStreamsWaitTime = ros::Duration(duration);

  this->pnh.param("transforms_update_period", duration, 1.0);
  this->transformsUpdatePeriod = ros::Duration(duration);

  this->buffer =
      std::make_unique<tf2_ros::Buffer>(ros::Duration(buffer_size), publish_frame_service);
  this->listener = std::make_unique<tf2_ros::TransformListener>(*buffer, this->nh);
  this->server = std::make_unique<tf2_ros::BufferServer>(*buffer, node_name, false);

  this->requestTransformStreamServer =
      pnh.advertiseService("request_transform_stream", &TF2Server::onRequestTransformStream, this);

  this->buffer->_addTransformsChangedListener(boost::bind(&TF2Server::updateFramesLists, this));

  if (this->pnh.hasParam("streams"))
  {
    XmlRpc::XmlRpcValue initialStreamsParam;
    this->pnh.getParam("streams", initialStreamsParam);
    if (initialStreamsParam.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Parameter streams has to be a dictionary");
      return;
    }

    for (const auto initialStream : initialStreamsParam)
    {
      const std::string streamName = initialStream.first;

      if (initialStream.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("The definition of stream %s has to be a dictionary", streamName.c_str());
        continue;
      }

      auto streamDef = initialStream.second;
      RequestTransformStreamRequest req;
      req.requested_topic_name = streamName;
      req.publisher_queue_size = 10;

      if (streamDef.hasMember("parent_frame"))
        req.parent_frame = static_cast<std::string>(streamDef["parent_frame"]);
      if (streamDef.hasMember("intermediate_frames"))
        req.intermediate_frames = static_cast<bool>(streamDef["intermediate_frames"]);
      if (streamDef.hasMember("publication_period"))
      {
        double period;
        if (streamDef["publication_period"].getType() == XmlRpc::XmlRpcValue::TypeInt)
          period = static_cast<int>(streamDef["publication_period"]);
        else if (streamDef["publication_period"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          period = static_cast<double>(streamDef["publication_period"]);
        req.publication_period = ros::Duration(period);
      }
      if (streamDef.hasMember("publisher_queue_size"))
        req.publisher_queue_size = static_cast<int>(streamDef["publisher_queue_size"]);
      if (streamDef.hasMember("allow_transforms_update"))
        req.allow_transforms_update = static_cast<bool>(streamDef["allow_transforms_update"]);
      if (streamDef.hasMember("child_frames"))
      {
        if (streamDef["child_frames"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("child_frames of stream %s have to be an array", streamName.c_str());
          continue;
        }

        for (size_t i = 0; i < streamDef["child_frames"].size(); ++i)
        {
          req.child_frames.push_back(streamDef["child_frames"][i]);
        }
      }

      this->initialStreams.push_back(req);
    }
  }
}

void TF2Server::start()
{
  this->server->start();
  this->started = true;
  this->lastTransformsUpdateTime = ros::Time::now();

  ROS_INFO("TF2 server started.");

  if (this->initialStreams.size() > 0)
  {
    this->initialStreamsTimer = this->pnh.createTimer(
      this->initialStreamsWaitTime, std::bind(&TF2Server::registerInitialStreams, this), true, true);
  }
}

bool operator==(const tf2_msgs::TFMessage& lhs, const tf2_msgs::TFMessage& rhs)
{
  if (lhs.transforms.size() != rhs.transforms.size())
    return false;

  for (size_t i = 0; i < lhs.transforms.size(); ++i)
  {
    const auto& lt = lhs.transforms[i];
    const auto& rt = rhs.transforms[i];

    if (lt.header.stamp != rt.header.stamp) return false;
    if (lt.header.frame_id != rt.header.frame_id) return false;
    if (lt.child_frame_id != rt.child_frame_id) return false;
    if (lt.transform.translation.x != rt.transform.translation.x) return false;
    if (lt.transform.translation.y != rt.transform.translation.y) return false;
    if (lt.transform.translation.z != rt.transform.translation.z) return false;
    if (lt.transform.rotation.x != rt.transform.rotation.x) return false;
    if (lt.transform.rotation.y != rt.transform.rotation.y) return false;
    if (lt.transform.rotation.z != rt.transform.rotation.z) return false;
    if (lt.transform.rotation.w != rt.transform.rotation.w) return false;
  }

  return true;
}

bool operator!=(const tf2_msgs::TFMessage& lhs, const tf2_msgs::TFMessage& rhs)
{
  return !(lhs == rhs);
}

bool TF2Server::onRequestTransformStream(RequestTransformStreamRequest &req,
                                         RequestTransformStreamResponse &resp)
{
  TopicsSpec topics;
  {
    std::lock_guard<std::mutex> streamsLock(this->streamsMutex);

    topics = this->getTopicsNames(req);
    if (topics.first.empty() || topics.second.empty())
      return false;

    this->streams[topics] = req;
  }

  const auto topicName = resp.topic_name = topics.first;
  const auto staticTopicName = resp.static_topic_name = topics.second;

  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->frames.find(req) == this->frames.end())
  {
    auto framesList = this->getFramesList(req);
    if (framesList->empty() && !req.allow_transforms_update)
      throw std::runtime_error("Could not find any child frames of frame " + req.parent_frame);

    this->frames[req] = std::move(framesList);
  }

  if (this->publishers.find(topicName) == this->publishers.end())
  {
    this->publishers[topicName] =
        this->nh.advertise<tf2_msgs::TFMessage>(topicName, req.publisher_queue_size,
          std::bind(&TF2Server::onSubscriberConnected, this, topics),
          std::bind(&TF2Server::onSubscriberDisconnected, this, topics));
  }

  if (this->staticPublishers.find(staticTopicName) == this->staticPublishers.end())
  {
    this->staticPublishers[staticTopicName] =
        this->nh.advertise<tf2_msgs::TFMessage>(staticTopicName, req.publisher_queue_size,
          std::bind(&TF2Server::onSubscriberConnected, this, topics),
          std::bind(&TF2Server::onSubscriberDisconnected, this, topics),
          ros::VoidConstPtr(), true);
  }

  if (this->timers.find(topics) == this->timers.end())
  {
    this->timers[topics] = this->nh.createTimer(req.publication_period,
      std::bind(&TF2Server::streamTransform, this, std::placeholders::_1, req, topics),
      false, false);
  }

  return true;
}

void TF2Server::streamTransform(const ros::TimerEvent &,
                                const RequestTransformStreamRequest &request,
                                const TopicsSpec& topics)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->frames[request]->empty())
    return;

  const ros::Duration timeout(request.publication_period.toSec() * 0.9 / this->frames[request]->size());
  tf2_msgs::TFMessage msg;
  tf2_msgs::TFMessage staticMsg;

  for (const auto& frame : *this->frames[request])
  {
    try
    {
      const auto transform = this->buffer->lookupTransform(frame.first, frame.second, ros::Time(0), timeout);
      if (transform.header.stamp != ros::Time(0, 0)) // static transforms are returned with zero timestamp
        msg.transforms.push_back(transform);
      else
        staticMsg.transforms.push_back(transform);
    }
    catch (tf2::TransformException &e)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Error publishing transform stream: " << e.what());
    }
  }

  this->publishers[topics.first].publish(msg);
  if (staticMsg != this->lastStaticTransforms[topics.second])
  {
    this->staticPublishers[topics.second].publish(staticMsg);
    this->lastStaticTransforms[topics.second] = staticMsg;
  }
}

TF2Server::TopicsSpec TF2Server::getTopicsNames(const RequestTransformStreamRequest &request)
{
  // make sure this->mutex is locked

  const RequestComparator comp;

  if (request.requested_topic_name.empty() && !request.requested_static_topic_name.empty())
  {
    throw std::runtime_error("when requested_static_topic_name is filled, "
                             "requested_topic_name has to be filled, too");
  }
  else if (!request.requested_topic_name.empty())
  {
    // if the request requests specific topic names, we have to do a few checks

    const auto topicName = this->pnh.resolveName(request.requested_topic_name);
    // if static topic name is not filled, just append static to the topic name
    const auto staticTopicName = this->pnh.resolveName(
        !request.requested_static_topic_name.empty() ? request.requested_static_topic_name :
        ros::names::append(request.requested_topic_name, "static"));
    const TopicsSpec topics = std::make_pair(topicName, staticTopicName);

    const auto existingStream = this->streams.find(topics);
    if (existingStream == this->streams.end())
    {
      // no stream exists yet
      this->topicNames.insert(topicName);
      this->topicNames.insert(staticTopicName);
      return topics;
    }
    else
    {
      // a stream on these topics already exists, check if it is compatible
      const auto existingRequest = (*existingStream).second;
      const auto streamsCompatible = comp.equals(request, existingRequest);

      if (streamsCompatible)
        return topics;
      else
        throw std::runtime_error(std::string("TF stream requested on topics ") + topicName +
          " and " + staticTopicName + "is not compatible with the already existing stream on these topics.");
    }
  }

  // try to find an existing compatible stream
  for (const auto& stream : this->streams)
  {
    const auto& topics = stream.first;
    const auto& existingRequest = stream.second;

    const auto streamsCompatible = comp.equals(request, existingRequest);

    if (streamsCompatible)
      return topics;
  }

  const auto baseName = this->pnh.resolveName(
      ros::names::append("streams", stripLeadingSlash(request.parent_frame, true)));
  std::string error;

  TopicsSpec topics = std::make_pair(std::string(""), std::string(""));

  bool topicNameFound = false;
  for (size_t i = 0; i < 10000; ++i)
  {
    topics.first = ros::names::append(baseName, "stream_" + std::to_string(i));
    error = "";
    if (ros::names::validate(topics.first, error) && this->topicNames.find(topics.first) == this->topicNames.end())
    {
      topicNameFound = true;
      break;
    }
  }

  if (!topicNameFound)
  {
    ROS_ERROR("Could not generate topic name for transform stream. Name validation error of last "
              "tried name '%s' is: %s", topics.first.c_str(), error.c_str());
    topics.first = topics.second = "";
    return topics;
  }

  // first try just appending "static"
  topics.second = ros::names::append(topics.first, "static");
  if (this->topicNames.find(topics.second) == this->topicNames.end())
  {
    this->topicNames.insert(topics.first);
    this->topicNames.insert(topics.second);
    return topics;
  }

  // otherwise, try again searching for the name iteratively

  bool staticTopicNameFound = false;
  for (size_t i = 0; i < 10000; ++i)
  {
    topics.second = ros::names::append(baseName, ros::names::append("stream_" + std::to_string(i), "static"));
    error = "";
    if (ros::names::validate(topics.second, error) && this->topicNames.find(topics.second) == this->topicNames.end())
    {
      staticTopicNameFound = true;
      break;
    }
  }

  if (!staticTopicNameFound)
  {
    ROS_ERROR("Could not generate static topic name for transform stream. Name validation error of last "
              "tried name '%s' is: %s", topics.second.c_str(), error.c_str());
    topics.first = topics.second = "";
    return topics;
  }

  this->topicNames.insert(topics.first);
  this->topicNames.insert(topics.second);

  return topics;
}

std::unique_ptr<TF2Server::FramesList> TF2Server::getFramesList(const RequestTransformStreamRequest &req) const
{
  auto result = std::make_unique<TF2Server::FramesList>();

  // if parent frame doesn't exist, we either wait for it (if it's allowed), or fail immediately
  if (!this->buffer->_frameExists(req.parent_frame))
  {
    if (!req.allow_transforms_update)
      throw tf2::LookupException("Frame " + req.parent_frame + " doesn't exist.");
    else
      return result;
  }

  if (!req.intermediate_frames)
  { // if intermediate frames are not requested, we just publish pairs of parent and all children
    result->reserve(req.child_frames.size());
    for (const auto& child : req.child_frames)
    {
      if (!this->buffer->_frameExists(child))
      {
        if (!req.allow_transforms_update)
          ROS_WARN("Frame %s doesn't exist, it won't be streamed.", child.c_str());
        continue;
      }
      result->emplace_back(req.parent_frame, child);
    }
  }
  else
  { // otherwise, we need to get a whole subtree
    std::vector<std::string> chainFrames;
    std::vector<std::string> childFrames;

    if (!req.child_frames.empty())
    { // only subtree to specific children is requested
      for (const auto& child : req.child_frames)
      {
        if (!this->buffer->_frameExists(child))
        {
          if (!req.allow_transforms_update)
            ROS_WARN("Frame %s doesn't exist, it won't be streamed.", child.c_str());
          continue;
        }
        childFrames.push_back(child);
      }
    }
    else
    { // whole subtree is requested; we don't know what children it has, so we have to find them
      std::vector<std::string> allFrames;
      this->buffer->_getFrameStrings(allFrames);

      std::string parentParent; // parent frame of req.parent_frame
      if (!this->buffer->_getParent(req.parent_frame, ros::Time(0), parentParent))
        parentParent = "";  // if req.parent_frame is the topmost frame

      for (const auto& frame : allFrames)
      {
        if (frame == req.parent_frame)
          continue;

        chainFrames.clear();
        try
        {
          this->buffer->_chainAsVector(frame, ros::Time(0), req.parent_frame, ros::Time(0), req.parent_frame, chainFrames);
        }
        catch (tf2::ExtrapolationException& e)
        {
          // the frame is stale, ignore it
          continue;
        }
        catch (tf2::TransformException& e)
        {
          ROS_ERROR("Error while searching TF tree: %s", e.what());
          continue;
        }

        // if chainFrames[1] is parentParent, it means that the path to the given frame does not
        // descend into req.parent_frame's subtree, but it goes above; we throw away such paths
        if (chainFrames[1] != parentParent)
          childFrames.push_back(frame);
      }
    }

    // collect all unique pairs of parent-child in the requested subtree

    std::set<TF2Server::FrameSpec> framePairs;
    for (const auto& child : childFrames)
    {
      chainFrames.clear();
      this->buffer->_chainAsVector(child, ros::Time(0), req.parent_frame, ros::Time(0), req.parent_frame, chainFrames);

      if (chainFrames.size() <= 1)
        continue;

      for (size_t i = 1; i < chainFrames.size(); ++i)
        framePairs.emplace(chainFrames[i-1], chainFrames[i]);
    }

    result->insert(result->begin(), framePairs.begin(), framePairs.end());
  }
  return result;
}

void TF2Server::updateFramesLists()
{
  // check update frequency
  if ((ros::Time::now() - this->lastTransformsUpdateTime) < this->transformsUpdatePeriod)
    return;
  this->lastTransformsUpdateTime = ros::Time::now();

  {
    std::lock_guard<std::mutex> lock(this->mutex);

    // update all frames that allow it
    for (auto &frame : this->frames)
    {
      const auto &req = frame.first;
      if (req.allow_transforms_update)
      {
        this->frames[req] = std::move(this->getFramesList(req));
      }
    }
  }
}

void TF2Server::onSubscriberConnected(const TopicsSpec& topics)
{
  std::lock_guard<std::mutex> lock(this->subscriberMutex);

  this->subscriberNumbers[topics] = this->subscriberNumbers[topics] + 1;
  if (this->subscriberNumbers[topics] == 1)
    ROS_INFO("Started streaming %s, %s", topics.first.c_str(), topics.second.c_str());

  this->timers[topics].start();
}

void TF2Server::onSubscriberDisconnected(const TopicsSpec& topics)
{
  std::lock_guard<std::mutex> lock(this->subscriberMutex);

  this->subscriberNumbers[topics] = this->subscriberNumbers[topics] - 1;
  if (this->subscriberNumbers[topics] == 0)
  {
    ROS_INFO("Stopped streaming %s, %s", topics.first.c_str(), topics.second.c_str());
    this->timers[topics].stop();
  }
}

void TF2Server::registerInitialStreams()
{
  for (auto& req : this->initialStreams)
  {
    try
    {
      RequestTransformStreamResponse resp;
      const auto succeeded = this->onRequestTransformStream(req, resp);
      if (succeeded)
        ROS_INFO("Stream %s, %s ready", resp.topic_name.c_str(), resp.static_topic_name.c_str());
      else
        ROS_ERROR("There was an error setting up transform stream %s", req.requested_topic_name.c_str());
    }
    catch (std::runtime_error &e)
    {
      ROS_ERROR("There was an error setting up transform stream %s: %s", req.requested_topic_name.c_str(), e.what());
    }
  }
}

bool RequestComparatorByFrames::operator()(
    const tf2_server::RequestTransformStreamRequest &r1,
    const tf2_server::RequestTransformStreamRequest &r2) const
{
  if (r1.intermediate_frames != r2.intermediate_frames)
    return r1.intermediate_frames < r2.intermediate_frames;
  else if (r1.allow_transforms_update != r2.allow_transforms_update)
    return r1.allow_transforms_update < r2.allow_transforms_update;
  else if (r1.parent_frame != r2.parent_frame)
    return r1.parent_frame < r2.parent_frame;
  else if (r1.child_frames.size() != r2.child_frames.size())
    return r1.child_frames.size() < r2.child_frames.size();
  else
  {
    for (size_t i = 0; i < r1.child_frames.size(); ++i)
    {
      if (r1.child_frames[i] != r2.child_frames[i])
        return r1.child_frames[i] < r2.child_frames[i];
    }
    return false;
  }
}

bool RequestComparatorByFrames::equals(
    const tf2_server::RequestTransformStreamRequest &r1,
    const tf2_server::RequestTransformStreamRequest &r2) const
{
  return !this->operator()(r1, r2) && !this->operator()(r2, r1);
}

bool RequestComparator::operator()(
    const tf2_server::RequestTransformStreamRequest &r1,
    const tf2_server::RequestTransformStreamRequest &r2) const
{
  if (r1.intermediate_frames != r2.intermediate_frames)
    return r1.intermediate_frames < r2.intermediate_frames;
  else if (r1.allow_transforms_update != r2.allow_transforms_update)
    return r1.allow_transforms_update < r2.allow_transforms_update;
  else if (r1.publication_period != r2.publication_period)
    return r1.publication_period < r2.publication_period;
  else if (r1.publisher_queue_size != r2.publisher_queue_size)
    return r1.publisher_queue_size < r2.publisher_queue_size;
  else if (r1.parent_frame != r2.parent_frame)
    return r1.parent_frame < r2.parent_frame;
  else if (r1.child_frames.size() != r2.child_frames.size())
    return r1.child_frames.size() < r2.child_frames.size();
  else
  {
    for (size_t i = 0; i < r1.child_frames.size(); ++i)
    {
      if (r1.child_frames[i] != r2.child_frames[i])
        return r1.child_frames[i] < r2.child_frames[i];
    }
    return false;
  }
  // requested_topic_name and requested_static_topic_name are omitted intentionally
}

bool RequestComparator::equals(
    const tf2_server::RequestTransformStreamRequest &r1,
    const tf2_server::RequestTransformStreamRequest &r2) const
{
  return !this->operator()(r1, r2) && !this->operator()(r2, r1);
}

}