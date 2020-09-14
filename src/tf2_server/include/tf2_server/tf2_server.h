#ifndef TF2_SERVER_TF2_SERVER_H
#define TF2_SERVER_TF2_SERVER_H

#include <map>
#include <memory>
#include <mutex>
#include <unordered_set>

#include <tf2_ros/buffer_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_server/RequestTransformStream.h>
#include <ros/ros.h>

namespace tf2_server
{

struct RequestComparatorByFrames
{
  bool operator()(const tf2_server::RequestTransformStreamRequest& r1,
                  const tf2_server::RequestTransformStreamRequest& r2) const;
  bool equals(const tf2_server::RequestTransformStreamRequest& r1,
              const tf2_server::RequestTransformStreamRequest& r2) const;
};

struct RequestComparator
{
  bool operator()(const tf2_server::RequestTransformStreamRequest& r1,
                  const tf2_server::RequestTransformStreamRequest& r2) const;
  bool equals(const tf2_server::RequestTransformStreamRequest& r1,
              const tf2_server::RequestTransformStreamRequest& r2) const;
};

class TF2Server
{

  protected: ros::NodeHandle& nh;
  protected: ros::NodeHandle& pnh;

  protected: std::unique_ptr<tf2_ros::Buffer> buffer;
  protected: std::unique_ptr<tf2_ros::BufferServer> server;
  protected: std::unique_ptr<tf2_ros::TransformListener> listener;
  protected: std::mutex mutex;
  protected: std::mutex subscriberMutex;
  protected: std::mutex streamsMutex;

  protected: ros::ServiceServer requestTransformStreamServer;

  protected: std::unordered_set<std::string> topicNames;
  protected: std::map<std::string, ros::Publisher> publishers;
  protected: std::map<std::string, ros::Publisher> staticPublishers;
  protected: std::map<std::string, tf2_msgs::TFMessage> lastStaticTransforms;

  protected: typedef std::pair<std::string, std::string> FrameSpec;
  protected: typedef std::vector<FrameSpec> FramesList;
  protected: std::map<RequestTransformStreamRequest, std::unique_ptr<FramesList>, RequestComparatorByFrames> frames;

  protected: typedef std::pair<std::string, std::string> TopicsSpec;
  protected: std::map<TopicsSpec, ros::Timer> timers;
  protected: std::map<TopicsSpec, size_t> subscriberNumbers;
  protected: std::map<TopicsSpec, RequestTransformStreamRequest> streams;

  protected: std::vector<RequestTransformStreamRequest> initialStreams;
  protected: ros::Duration initialStreamsWaitTime;
  protected: ros::Timer initialStreamsTimer;

  protected: ros::Time lastTransformsUpdateTime;
  protected: ros::Duration transformsUpdatePeriod;

  protected: bool started = false;

  public: explicit TF2Server(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  public: virtual void start();

  protected: void registerInitialStreams();

  protected: virtual bool onRequestTransformStream(RequestTransformStreamRequest& req, RequestTransformStreamResponse& resp);

  protected: virtual void streamTransform(const ros::TimerEvent& event, const RequestTransformStreamRequest& request, const TopicsSpec& topics);

  protected: virtual std::unique_ptr<FramesList> getFramesList(const RequestTransformStreamRequest& req) const;

  protected: virtual TopicsSpec getTopicsNames(const RequestTransformStreamRequest& request);

  protected: virtual void updateFramesLists();

  protected: virtual void onSubscriberConnected(const TopicsSpec& topics);
  protected: virtual void onSubscriberDisconnected(const TopicsSpec& topics);

};

}

#endif //TF2_SERVER_TF2_SERVER_H
