// TODO if https://github.com/ros/robot_state_publisher/pull/106 is merged, this hack can be left out
// HACK we cannot substitute the RobotStatePublisher in JointStateListener, so we hack it like this
#define protected public
#include <robot_state_publisher/robot_state_publisher.h>
#undef protected

#include "dynamic_robot_state_publisher/robot_state_publisher.h"

std::string stripSlash(const std::string & in)
{
  if (!in.empty() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

void robot_state_publisher::DynamicRobotStatePublisher::updateTree(const KDL::Tree &tree)
{
  /// function is only called from JointStateListener::reload_robot_model
  /// where the update mutex is acquired

  auto old_segments_fixed = publisher->segments_fixed_;

  publisher->segments_.clear();
  publisher->segments_fixed_.clear();
  publisher->addChildren(tree.getRootSegment());

  // find out which segments have disappeared and publish a special TF message
  // that disconnects them from the main TF tree

  for (const auto& seg : publisher->segments_fixed_)
  {
    old_segments_fixed.erase(seg.first);
  }
  for (const auto& seg : publisher->segments_)
  {
    old_segments_fixed.erase(seg.first);
  }

  std::vector<geometry_msgs::TransformStamped> deleteTfs;
  for (const auto& seg : old_segments_fixed)
  {
    geometry_msgs::TransformStamped tf;
    tf.child_frame_id = stripSlash(seg.second.tip);
    tf.header.frame_id = DynamicRobotStatePublisher::DELETED_STATIC_TFS_FRAME;
    tf.header.stamp = ros::Time::now();
    tf.transform.rotation.w = 1.0;
    deleteTfs.push_back(tf);
  }
  publisher->static_tf_broadcaster_.sendTransform(deleteTfs);
}

robot_state_publisher::DynamicRobotStatePublisher::DynamicRobotStatePublisher(
  robot_state_publisher::RobotStatePublisher *publisher) : publisher(publisher)
{}

size_t robot_state_publisher::DynamicRobotStatePublisher::getNumMovingJoints() const
{
  return publisher->segments_.size();
}

size_t robot_state_publisher::DynamicRobotStatePublisher::getNumFixedJoints() const
{
  return publisher->segments_fixed_.size();
}
