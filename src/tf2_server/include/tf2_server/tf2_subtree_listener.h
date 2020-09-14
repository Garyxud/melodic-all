#ifndef TF2_SERVER_TF2_SUBTREE_LISTENER_H
#define TF2_SERVER_TF2_SUBTREE_LISTENER_H

#include <tf2_ros/transform_listener.h>
#include <tf2_server/RequestTransformStreamRequest.h>

#include <memory>

namespace tf2_server
{

class TransformSubtreeListener
{
  public: TransformSubtreeListener(const tf2_server::RequestTransformStreamRequest& subtree, tf2::BufferCore &buffer, bool spinThread = true, ros::Duration maxServerWait = ros::Duration(-1));
  public: TransformSubtreeListener(const tf2_server::RequestTransformStreamRequest& subtree, tf2::BufferCore &buffer, const ros::NodeHandle &nh, bool spinThread = true, ros::Duration maxServerWait = ros::Duration(-1));
  public: void updateSubtree(const tf2_server::RequestTransformStreamRequest& subtree);

  protected: std::unique_ptr<tf2_ros::TransformListener> listener;
  protected: ros::ServiceClient requestTransformStream;
  protected: tf2::BufferCore& buffer;
  protected: ros::NodeHandle nh;
  protected: bool spinThread;
};

}

#endif //TF2_SERVER_TF2_SUBTREE_LISTENER_H
