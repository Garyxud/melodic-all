#include <gtest/gtest.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <dataspeed_ulc_msgs/UlcReport.h>
#include <dataspeed_ulc_can/dispatch.h>
#include <can_msgs/Frame.h>
#include "MsgRx.h"

using namespace dataspeed_ulc_can;

ros::NodeHandle *n, *pn;

MsgRx<MsgUlcCmd> g_msg_ulc_cmd(ros::WallDuration(0.05));
MsgRx<MsgUlcCfg> g_msg_ulc_cfg(ros::WallDuration(0.05));
MsgRx<dataspeed_ulc_msgs::UlcReport> g_msg_ulc_report(ros::WallDuration(0.05));

dataspeed_ulc_msgs::UlcCmd g_ulc_cmd;
double g_cfg_freq;

// Command message scale factors
const double LIN_VEL_SCALE_FACTOR = 0.0025;
const double YAW_RATE_SCALE_FACTOR = 0.00025;
const double CURVATURE_SCALE_FACTOR = 0.0000061;

// Config message scale factors
const double LINEAR_ACCEL_SCALE_FACTOR = 0.025;
const double LINEAR_DECEL_SCALE_FACTOR = 0.025;
const double LATERAL_ACCEL_SCALE_FACTOR = 0.05;
const double ANGULAR_ACCEL_SCALE_FACTOR = 0.02;

// Report message scale factors
const double SPEED_REPORT_SCALE_FACTOR = 0.02;
const double ACCEL_REPORT_SCALE_FACTOR = 0.05;
const double MAX_ANGLE_SCALE_FACTOR = 5.0;
const double MAX_RATE_SCALE_FACTOR = 8.0;

ros::Publisher g_pub_ulc_cmd;
ros::Publisher g_pub_enable;
ros::Publisher g_pub_twist;
ros::Publisher g_pub_twist_stamped;
ros::Publisher g_pub_can;
ros::Subscriber g_sub_can;
ros::Subscriber g_sub_report;

void recvCan(const can_msgs::FrameConstPtr& msg)
{
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_ULC_CMD:
        g_msg_ulc_cmd.set(*((MsgUlcCmd*)msg->data.elems));
        break;
      case ID_ULC_CONFIG:
        g_msg_ulc_cfg.set(*((MsgUlcCfg*)msg->data.elems));
        break;
    }
  }
}

void recvReport(const dataspeed_ulc_msgs::UlcReportConstPtr& msg)
{
  g_msg_ulc_report.set(*msg);
}

template <class T>
static bool waitForMsg(ros::WallDuration dur, const MsgRx<T>& msg_rx)
{
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if (msg_rx.fresh()) {
      return true;
    }
    if ((ros::WallTime::now() - start) > dur) {
      return false;
    }
    ros::WallDuration(0.001).sleep();
  }
}

static bool waitForTopics(ros::WallDuration dur) {
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if ((g_sub_can.getNumPublishers() == 1)
     && (g_sub_report.getNumPublishers() == 1)
     && (g_pub_ulc_cmd.getNumSubscribers() == 1)
     && (g_pub_enable.getNumSubscribers() == 1)
     && (g_pub_twist.getNumSubscribers() == 1)
     && (g_pub_twist_stamped.getNumSubscribers() == 1)
     && (g_pub_can.getNumSubscribers() == 1)) {
      return true;
    }
    if ((ros::WallTime::now() - start) > dur) {
      return false;
    }
    ros::WallDuration(0.001).sleep();
  }
}

static void checkImmediateCfg()
{
  ros::WallTime stamp = ros::WallTime::now();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
  EXPECT_NEAR(g_msg_ulc_cfg.stamp().toSec(), stamp.toSec(), 0.01);
}

TEST(ULCNode, topics)
{
  // Wait for all topics to connect before running other tests
  ASSERT_TRUE(waitForTopics(ros::WallDuration(2.0)));
  ros::WallDuration(1.0).sleep();

  // Call unused function to complete coverage testing
  dispatchAssertSizes();
}

TEST(ULCNode, cfgTiming)
{
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.linear_accel = 1.0;
  g_ulc_cmd.linear_decel = 1.0;
  g_ulc_cmd.lateral_accel = 1.0;
  g_ulc_cmd.angular_accel = 1.0;

  // Publish command messages with the same acceleration limits and make sure
  // config CAN messages are sent at the nominal rate
  size_t count = 0;
  ros::WallTime stamp_old;
  ros::WallTime t_end = ros::WallTime::now() + ros::WallDuration(1.0);
  g_msg_ulc_cfg.clear();
  while (t_end > ros::WallTime::now()) {
    g_pub_ulc_cmd.publish(g_ulc_cmd);
    ros::WallDuration(0.02).sleep();
    ros::WallTime stamp_new = g_msg_ulc_cfg.stamp();
    if (!stamp_new.isZero()) {
      if (!stamp_old.isZero() && (stamp_old != stamp_new)) {
        EXPECT_NEAR(stamp_old.toSec() + (1.0 / g_cfg_freq), stamp_new.toSec(), 0.01);
        count++;
      }
      stamp_old = stamp_new;
    }
  }
  EXPECT_GE(count, 1);

  // Change accel limits and make sure config CAN messages are sent immediately
  g_ulc_cmd.linear_accel = 2.0;
  checkImmediateCfg();
  g_ulc_cmd.linear_decel = 2.0;
  checkImmediateCfg();
  g_ulc_cmd.lateral_accel = 2.0;
  checkImmediateCfg();
  g_ulc_cmd.angular_accel = 2.0;
  checkImmediateCfg();
}

TEST(ULCNode, cmdRangeSaturation)
{
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();

  /*** Underflow tests ******************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = (INT16_MIN * LIN_VEL_SCALE_FACTOR) - 1.0;
  g_ulc_cmd.linear_accel = -1.0;
  g_ulc_cmd.linear_decel = -1.0;
  g_ulc_cmd.lateral_accel = -1.0;
  g_ulc_cmd.angular_accel = -1.0;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = (INT16_MIN * YAW_RATE_SCALE_FACTOR) - 0.5;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = (INT16_MIN * CURVATURE_SCALE_FACTOR) - 0.05;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/

  /*** Overflow tests *******************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = (INT16_MAX * LIN_VEL_SCALE_FACTOR) + 1.0;
  g_ulc_cmd.linear_accel = 100.0;
  g_ulc_cmd.linear_decel = 100.0;
  g_ulc_cmd.lateral_accel = 100.0;
  g_ulc_cmd.angular_accel = 100.0;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = (INT16_MAX * YAW_RATE_SCALE_FACTOR) + 0.5;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = (INT16_MAX * CURVATURE_SCALE_FACTOR) + 0.05;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/

  /*** +Inf tests ***********************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = INFINITY;
  g_ulc_cmd.linear_accel = INFINITY;
  g_ulc_cmd.linear_decel = INFINITY;
  g_ulc_cmd.lateral_accel = INFINITY;
  g_ulc_cmd.angular_accel = INFINITY;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/

  /*** -Inf tests ***********************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = -INFINITY;
  g_ulc_cmd.linear_accel = -INFINITY;
  g_ulc_cmd.linear_decel = -INFINITY;
  g_ulc_cmd.lateral_accel = -INFINITY;
  g_ulc_cmd.angular_accel = -INFINITY;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = -INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = -INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/
}

TEST(ULCNode, outOfBoundsInputs)
{
  g_msg_ulc_cfg.clear();

  // NaN in linear velocity field
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.linear_velocity = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // NaN in yaw command field
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.yaw_command = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // NaN in linear accel field
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.linear_accel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // NaN in linear decel field
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.linear_decel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // NaN in lateral accel field
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.lateral_accel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // NaN in angular accel field
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.angular_accel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // Invalid steering mode
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.steering_mode = 3;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));

  // Make sure no config messages were sent during this process
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
}

TEST(ULCNode, scaleFactors)
{
  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.linear_velocity = 22.3;
  g_ulc_cmd.linear_accel = 1.23;
  g_ulc_cmd.linear_decel = 3.45;
  g_ulc_cmd.lateral_accel = 5.43;
  g_ulc_cmd.angular_accel = 3.21;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = 0.567;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cfg));
  EXPECT_EQ((int16_t)(g_ulc_cmd.linear_velocity / LIN_VEL_SCALE_FACTOR), g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ((int16_t)(g_ulc_cmd.yaw_command / YAW_RATE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ((int16_t)(g_ulc_cmd.linear_accel / LINEAR_ACCEL_SCALE_FACTOR), g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ((int16_t)(g_ulc_cmd.linear_decel / LINEAR_DECEL_SCALE_FACTOR), g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ((int16_t)(g_ulc_cmd.lateral_accel / LATERAL_ACCEL_SCALE_FACTOR), g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ((int16_t)(g_ulc_cmd.angular_accel / ANGULAR_ACCEL_SCALE_FACTOR), g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = 0.0789;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_EQ((int16_t)(g_ulc_cmd.yaw_command / CURVATURE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
}

TEST(ULCNode, dbwEnable)
{
  std_msgs::Bool dbw_enabled_msg;

  g_ulc_cmd = dataspeed_ulc_msgs::UlcCmd();
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.enable_shifting = true;
  g_ulc_cmd.shift_from_park = true;

  // Make sure CAN enable signals are false because dbw_enabled was not sent yet
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_FALSE(g_msg_ulc_cmd.get().enable_pedals);
  EXPECT_FALSE(g_msg_ulc_cmd.get().enable_steering);
  EXPECT_FALSE(g_msg_ulc_cmd.get().enable_shifting);
  EXPECT_FALSE(g_msg_ulc_cmd.get().shift_from_park);

  // Publish dbw_enabled as true
  dbw_enabled_msg.data = true;
  g_pub_enable.publish(dbw_enabled_msg);
  ros::WallDuration(0.001).sleep();

  // Send command again and make sure CAN enable signals are true
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_TRUE(g_msg_ulc_cmd.get().enable_pedals);
  EXPECT_TRUE(g_msg_ulc_cmd.get().enable_steering);
  EXPECT_TRUE(g_msg_ulc_cmd.get().enable_shifting);
  EXPECT_TRUE(g_msg_ulc_cmd.get().shift_from_park);

  // Publish dbw_enabled as false and make sure CAN enable signals are false
  dbw_enabled_msg.data = false;
  g_pub_enable.publish(dbw_enabled_msg);
  ros::WallDuration(0.05).sleep();
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd.publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  ASSERT_FALSE(g_msg_ulc_cmd.get().enable_pedals);
  ASSERT_FALSE(g_msg_ulc_cmd.get().enable_steering);
  ASSERT_FALSE(g_msg_ulc_cmd.get().enable_shifting);
  ASSERT_FALSE(g_msg_ulc_cmd.get().shift_from_park);
}

TEST(ULCNode, twistInputs)
{
  geometry_msgs::Twist twist_cmd;
  twist_cmd.linear.x = 22.0;
  twist_cmd.angular.z = 0.2;
  ros::WallDuration(1.0).sleep();

  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_twist.publish(twist_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.5), g_msg_ulc_cfg));
  EXPECT_EQ((int16_t)(twist_cmd.linear.x / LIN_VEL_SCALE_FACTOR), g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ((int16_t)(twist_cmd.angular.z / YAW_RATE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cmd.get().steering_mode);

  geometry_msgs::TwistStamped twist_stamped_cmd;
  twist_stamped_cmd.twist = twist_cmd;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_twist_stamped.publish(twist_stamped_cmd);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_cmd));
  EXPECT_FALSE(waitForMsg(ros::WallDuration(0.5), g_msg_ulc_cfg));
  EXPECT_EQ((int16_t)(twist_cmd.linear.x / LIN_VEL_SCALE_FACTOR), g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ((int16_t)(twist_cmd.angular.z / YAW_RATE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cmd.get().steering_mode);
}

TEST(ULCNode, reportParsing)
{
  can_msgs::Frame report_out;
  report_out.id = ID_ULC_REPORT;
  report_out.is_extended = false;
  report_out.dlc = sizeof(MsgUlcReport);
  MsgUlcReport* msg_report_ptr = (MsgUlcReport*)report_out.data.elems;
  memset(msg_report_ptr, 0x00, sizeof(MsgUlcReport));
  msg_report_ptr->timeout = false;
  msg_report_ptr->tracking_mode = 0;
  msg_report_ptr->steering_mode = 1;
  msg_report_ptr->steering_enabled = false;
  msg_report_ptr->pedals_enabled = true;
  msg_report_ptr->speed_ref = 22.2f / SPEED_REPORT_SCALE_FACTOR;
  msg_report_ptr->accel_ref = 1.1f / ACCEL_REPORT_SCALE_FACTOR;
  msg_report_ptr->speed_meas = 21.1f / SPEED_REPORT_SCALE_FACTOR;
  msg_report_ptr->accel_meas = 0.99f / ACCEL_REPORT_SCALE_FACTOR;
  msg_report_ptr->max_steering_vel = 16.0f / MAX_RATE_SCALE_FACTOR;
  msg_report_ptr->max_steering_angle = 55.0f / MAX_ANGLE_SCALE_FACTOR;
  msg_report_ptr->speed_preempted = true;
  msg_report_ptr->steering_preempted = false;
  msg_report_ptr->override = true;

  g_pub_can.publish(report_out);
  ASSERT_TRUE(waitForMsg(ros::WallDuration(0.1), g_msg_ulc_report));
  ASSERT_FALSE(g_msg_ulc_report.get().timeout);
  ASSERT_EQ(0, g_msg_ulc_report.get().tracking_mode);
  ASSERT_EQ(1, g_msg_ulc_report.get().steering_mode);
  ASSERT_FALSE(g_msg_ulc_report.get().steering_enabled);
  ASSERT_TRUE(g_msg_ulc_report.get().pedals_enabled);
  ASSERT_FLOAT_EQ(22.2f, g_msg_ulc_report.get().speed_ref);
  ASSERT_FLOAT_EQ(1.1f, g_msg_ulc_report.get().accel_ref);
  ASSERT_FLOAT_EQ(21.1f, g_msg_ulc_report.get().speed_meas);
  ASSERT_FLOAT_EQ(0.95f, g_msg_ulc_report.get().accel_meas);
  ASSERT_FLOAT_EQ(16.0f, g_msg_ulc_report.get().max_steering_vel);
  ASSERT_FLOAT_EQ(55.0f, g_msg_ulc_report.get().max_steering_angle);
  ASSERT_TRUE(g_msg_ulc_report.get().speed_preempted);
  ASSERT_FALSE(g_msg_ulc_report.get().steering_preempted);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ulc_node_test");
  n = new ros::NodeHandle();
  pn = new ros::NodeHandle("~");

  const ros::TransportHints NODELAY = ros::TransportHints().tcpNoDelay();
  g_sub_can = n->subscribe("can_tx", 100, recvCan, NODELAY);
  g_sub_report = n->subscribe("ulc_report", 10, recvReport, NODELAY);
  g_pub_ulc_cmd = n->advertise<dataspeed_ulc_msgs::UlcCmd>("ulc_cmd", 2);
  g_pub_enable = n->advertise<std_msgs::Bool>("dbw_enabled", 2);
  g_pub_twist = n->advertise<geometry_msgs::Twist>("cmd_vel", 2);
  g_pub_twist_stamped = n->advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 2);
  g_pub_can = n->advertise<can_msgs::Frame>("can_rx", 100);
  pn->param("config_frequency", g_cfg_freq, 5.0);

  // Setup Spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Run all the tests that were declared with TEST()
  int result = RUN_ALL_TESTS();

  // Cleanup
  spinner.stop();
  n->shutdown();
  pn->shutdown();

  // Return test result
  return result;
}
