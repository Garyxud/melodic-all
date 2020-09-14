/* HACK HACK HACK */
/* We want to access TransformListener members. */
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private public
#include <tf2_ros/transform_listener.h>
#undef private

#include <tf2_server/tf2_subtree_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gtest/gtest.h>

using namespace tf2_server;

class TestSubtreeListener : public TransformSubtreeListener
{

  public: TestSubtreeListener(const tf2_server::RequestTransformStreamRequest& subtree, tf2::BufferCore &buffer, bool spinThread = true, ros::Duration maxServerWait = ros::Duration(-1)) :
    TransformSubtreeListener(subtree, buffer, spinThread, maxServerWait)
  {
  }

  public: std::unique_ptr<tf2_ros::TransformListener>& getListener()
  {
    return this->listener;
  }
};

TEST(tf2_subtree_listener, Basic){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  req.intermediate_frames = false;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE( buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE( buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE( buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, PublicationPeriod){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  req.intermediate_frames = false;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(10);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, IntermediateFrames){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, Subtree){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, UpdateSubtree){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { "left_track" };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer;
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  req.child_frames = { "left_track", "front_left_flipper_endpoint" };
  listener.updateSubtree(req);

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, MultipleListeners){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  tf2_ros::Buffer buffer2(ros::Duration(1, 0));
  TransformSubtreeListener listener2(req, buffer2, false, ros::Duration(10));

  buffer.clear();
  ros::Duration(2).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  EXPECT_FALSE(buffer2.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer2.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer2.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, NamedStreams){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);
  req.requested_topic_name = "test";
  req.requested_static_topic_name = "test_static";

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  TestSubtreeListener listener(req, buffer, false, ros::Duration(10));

  EXPECT_EQ(listener.getListener()->message_subscriber_tf_.getTopic(), "/tf2_buffer_server/test");
  EXPECT_EQ(listener.getListener()->message_subscriber_tf_static_.getTopic(), "/tf2_buffer_server/test_static");

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  req.requested_topic_name = "test2";
  req.requested_static_topic_name = "test2_static";
  tf2_ros::Buffer buffer2(ros::Duration(1, 0));
  TestSubtreeListener listener2(req, buffer2, false, ros::Duration(10));

  EXPECT_EQ(listener2.getListener()->message_subscriber_tf_.getTopic(), "/tf2_buffer_server/test2");
  EXPECT_EQ(listener2.getListener()->message_subscriber_tf_static_.getTopic(), "/tf2_buffer_server/test2_static");

  buffer.clear();
  ros::Duration(2).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  EXPECT_FALSE(buffer2.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer2.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer2.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer2.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer2.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, NamedStreamDefaultStatic){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);
  req.requested_topic_name = "test";

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  TestSubtreeListener listener(req, buffer, false, ros::Duration(10));

  EXPECT_EQ(listener.getListener()->message_subscriber_tf_.getTopic(), "/tf2_buffer_server/test");
  EXPECT_EQ(listener.getListener()->message_subscriber_tf_static_.getTopic(), "/tf2_buffer_server/test/static");

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

TEST(tf2_subtree_listener, NamedStreamOnlyStaticFails)
{
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = {};
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);
  req.requested_static_topic_name = "test_static";

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  EXPECT_THROW(TransformSubtreeListener(req, buffer, false, ros::Duration(10)), tf2::InvalidArgumentException);
}

TEST(tf2_subtree_listener, ServerTimeout)
{
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = {};
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  const ros::M_string remappings = {{"tf2_server", "nonexistent"}};
  ros::NodeHandle nh("", remappings);
  EXPECT_THROW(TransformSubtreeListener(req, buffer, nh, false, ros::Duration(1)), tf2::TimeoutException);
}

TEST(tf2_subtree_listener, TestStreamsGetUpdated){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);
  req.allow_transforms_update = true;

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  TestSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "foo_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "foo_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("right_track", "foo_track", ros::Time(0)));

  auto broadcaster = tf2_ros::StaticTransformBroadcaster();
  auto tf = geometry_msgs::TransformStamped();
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "foo_track";
  tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0;
  tf.transform.rotation.x = tf.transform.rotation.y = tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  broadcaster.sendTransform(tf);

  ros::Duration(2).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "foo_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "foo_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("right_track", "foo_track", ros::Time(0)));
}

TEST(tf2_subtree_listener, TestStreamsDontGetUpdatedIfNotAllowed){
  RequestTransformStreamRequest req;
  req.parent_frame = "base_link";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);
  req.allow_transforms_update = false;

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  TestSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "foo2_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "foo2_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("right_track", "foo2_track", ros::Time(0)));

  auto broadcaster = tf2_ros::StaticTransformBroadcaster();
  auto tf = geometry_msgs::TransformStamped();
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "foo2_track";
  tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0;
  tf.transform.rotation.x = tf.transform.rotation.y = tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  broadcaster.sendTransform(tf);

  ros::Duration(2).sleep();

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "foo2_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "foo2_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("right_track", "foo2_track", ros::Time(0)));
}

TEST(tf2_subtree_listener, TestStreamsGetUpdatedWithMissingParent){
  RequestTransformStreamRequest req;
  req.parent_frame = "map";
  req.child_frames = { };
  req.intermediate_frames = true;
  req.publisher_queue_size = 10;
  req.publication_period = ros::Duration(0.1);
  req.allow_transforms_update = true;

  tf2_ros::Buffer buffer(ros::Duration(1, 0));
  TestSubtreeListener listener(req, buffer, false, ros::Duration(10));

  ros::Duration(1).sleep();

  EXPECT_FALSE(buffer.canTransform("map", "odom", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("map", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("map", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("map", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("map", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("map", "right_track", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_FALSE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_FALSE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));

  auto broadcaster = tf2_ros::StaticTransformBroadcaster();
  auto tf = geometry_msgs::TransformStamped();
  tf.header.frame_id = "map";
  tf.child_frame_id = "odom";
  tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0;
  tf.transform.rotation.x = tf.transform.rotation.y = tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  broadcaster.sendTransform(tf);

  ros::Duration(2).sleep();

  EXPECT_TRUE(buffer.canTransform("map", "odom", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("map", "base_link", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("map", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("map", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("map", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("map", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("odom", "base_link", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("odom", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("odom", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("odom", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("odom", "right_track", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "left_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("front_left_flipper", "front_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("left_track", "rear_left_flipper_endpoint", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("rear_left_flipper", "rear_left_flipper_endpoint", ros::Time(0)));

  EXPECT_TRUE(buffer.canTransform("base_link", "right_track", ros::Time(0)));
  EXPECT_TRUE(buffer.canTransform("base_link", "front_right_flipper", ros::Time(0)));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_tf2_subtree_listener");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}