/*
 * safe_trajectory_planner.cpp
 *
 *  Created on: Mar 25, 2010
 *      Author: duhadway
 */

#include <safe_teleop_base/safe_trajectory_planner_ros.h>
#include <ros/console.h>
#include <sys/time.h>

#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"

using namespace std;
using namespace costmap_2d;
using namespace base_local_planner;

namespace safe_teleop {

  SafeTrajectoryPlannerROS::SafeTrajectoryPlannerROS(TFListener* tf, Costmap2DROS* costmap_ros)
    : nh_(), world_model_(NULL), tc_(NULL), costmap_ros_(costmap_ros), tf_(tf) {
    rot_stopped_velocity_ = 1e-2;
    trans_stopped_velocity_ = 1e-2;
    double acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity;
    int vx_samples, vy_samples, vtheta_samples;
    double userdist_scale, occdist_scale;
    bool   dwa, holonomic_robot;
    double max_vel_x, min_vel_x;
    double max_vel_y, min_vel_y;
    string world_model_type;

    //initialize the copy of the costmap the controller will use
    //costmap_ros_->getCostmapCopy(costmap_);
    costmap_ = *costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~");

    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    u_plan_pub_ = private_nh.advertise<nav_msgs::Path>("user_plan", 1);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &SafeTrajectoryPlannerROS::odomCallback, this);
    user_sub_ = nh_.subscribe<geometry_msgs::Twist>("base_velocity", 1, boost::bind(&SafeTrajectoryPlannerROS::cmdCallback, this, _1));

    cmd_pub_ = private_nh.advertise<geometry_msgs::Twist>("safe_vel", 1);

    geometry_msgs::Twist vel;
    cmd_pub_.publish(vel);

    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &SafeTrajectoryPlannerROS::clearCostmapsService, this);

    //we'll get the parameters for the robot radius from the costmap we're associated with
    inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    circumscribed_radius_ = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();

    private_nh.param("acc_lim_x", acc_lim_x, 2.5);
    private_nh.param("acc_lim_y", acc_lim_y, 2.5);
    private_nh.param("acc_lim_th", acc_lim_theta, 3.2);
    private_nh.param("sim_time", sim_time, 1.0);
    private_nh.param("sim_granularity", sim_granularity, 0.025);
    private_nh.param("vx_samples", vx_samples, 3);
    private_nh.param("vy_samples", vy_samples, 5);
    private_nh.param("vtheta_samples", vtheta_samples, 10);
    private_nh.param("user_bias", userdist_scale, 0.6);
    private_nh.param("occdist_scale", occdist_scale, 0.01);
    private_nh.param("max_vel_x", max_vel_x, 0.5);
    private_nh.param("min_vel_x", min_vel_x, 0.1);
    private_nh.param("max_vel_y", max_vel_y, 0.2);
    private_nh.param("min_vel_y", min_vel_y, -0.2);


    double max_rotational_vel;
    private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
    max_vel_th_ = max_rotational_vel;
    min_vel_th_ = -1.0 * max_rotational_vel;

    private_nh.param("world_model", world_model_type, string("costmap"));
    private_nh.param("holonomic_robot", holonomic_robot, true);
    private_nh.param("dwa", dwa, true);
    private_nh.param("safe_backwards", safe_backwards_, false);

    //parameters for using the freespace controller
    double min_pt_separation, max_obstacle_height, grid_resolution;
    private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
    private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
    private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
    private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

    ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
    world_model_ = new CostmapModel(costmap_);

    std::vector<double> y_vels = loadYVels(private_nh);

    tc_ = new SafeTrajectoryPlanner(*world_model_, costmap_, costmap_ros_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_,
        acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity, vx_samples, vy_samples, vtheta_samples,
        userdist_scale, occdist_scale,
        max_vel_x, min_vel_x, max_vel_y, min_vel_y, max_vel_th_, min_vel_th_,
        holonomic_robot, dwa);
  }

  std::vector<double> SafeTrajectoryPlannerROS::loadYVels(ros::NodeHandle node){
    std::vector<double> y_vels;

    XmlRpc::XmlRpcValue y_vel_list;
    if(node.getParam("y_vels", y_vel_list)){
      ROS_ASSERT_MSG(y_vel_list.getType() == XmlRpc::XmlRpcValue::TypeArray,
          "The y velocities to explore must be specified as a list");

      for(int i = 0; i < y_vel_list.size(); ++i){
        //make sure we have a list of lists of size 2
        XmlRpc::XmlRpcValue vel = y_vel_list[i];

        //make sure that the value we're looking at is either a double or an int
        ROS_ASSERT(vel.getType() == XmlRpc::XmlRpcValue::TypeInt || vel.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double y_vel = vel.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(vel) : (double)(vel);

        y_vels.push_back(y_vel);

      }
    }
    else{
      //if no values are passed in, we'll provide defaults
      y_vels.push_back(-0.3);
      y_vels.push_back(-0.1);
      y_vels.push_back(0.1);
      y_vels.push_back(0.3);
    }

    return y_vels;
  }

  SafeTrajectoryPlannerROS::~SafeTrajectoryPlannerROS(){
    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  bool SafeTrajectoryPlannerROS::stopped(){
    boost::recursive_mutex::scoped_lock(odom_lock_);
    return abs(base_odom_.twist.twist.angular.z) <= rot_stopped_velocity_
      && abs(base_odom_.twist.twist.linear.x) <= trans_stopped_velocity_
      && abs(base_odom_.twist.twist.linear.y) <= trans_stopped_velocity_;
  }

  double SafeTrajectoryPlannerROS::distance(double x1, double y1, double x2, double y2){
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  void SafeTrajectoryPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::recursive_mutex::scoped_lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
//    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
//        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  void SafeTrajectoryPlannerROS::cmdCallback(const geometry_msgs::Twist::ConstPtr& vel) {
    if ((safe_backwards_  && ((fabs(vel->linear.x) > 0) || (fabs(vel->linear.y) > 0))) ||
        (!safe_backwards_ && ((vel->linear.x > 0) || (fabs(vel->linear.y) > 0))))
    {
      geometry_msgs::Twist safe_vel;
      if (computeVelocityCommands(vel, safe_vel)) {
        cmd_pub_.publish(safe_vel);
        if ((vel->linear.x != safe_vel.linear.x) || (vel->angular.z != safe_vel.angular.z)) {
          ROS_DEBUG("safe: (%.2f, %.2f) -> (%.2f, %.2f)",
              vel->linear.x, vel->angular.z,
              safe_vel.linear.x, safe_vel.angular.z);
        }
      } else {
        geometry_msgs::Twist zero_vel;
        cmd_pub_.publish(zero_vel);
        ROS_DEBUG("zero: (%.2f, %.2f) -> (%.2f, %.2f)",
            vel->linear.x, vel->angular.z,
            zero_vel.linear.x, zero_vel.angular.z);
      }

    } else { // backwards and rotations allowed to pass through directly
      cmd_pub_.publish(vel);
    }
  }

  bool SafeTrajectoryPlannerROS::computeVelocityCommands(const geometry_msgs::Twist::ConstPtr& vel, geometry_msgs::Twist& cmd_vel){
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> user_plan;
    tf::Stamped<tf::Pose> global_pose;
#if ROS_VERSION_MINIMUM(1, 14, 0) // ROS_MELODIC
    geometry_msgs::PoseStamped global_pose_stamped;
    if(!costmap_ros_->getRobotPose(global_pose_stamped))
      return false;
    tf::poseStampedMsgToTF(global_pose_stamped, global_pose);
#else
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;
#endif

    //we also want to clear the robot footprint from the costmap we're using
//    costmap_ros_->clearRobotFootprint();

    //make sure to update the costmap we'll use for this cycle
    costmap_ = *costmap_ros_->getCostmap();

    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;

    odom_lock_.lock();
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;
    odom_lock_.unlock();

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = robot_base_frame_;

    tf::Stamped<tf::Pose> robot_vel;

    robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
    robot_vel.frame_id_ = robot_base_frame_;
    robot_vel.stamp_ = ros::Time();

    tf::Stamped<tf::Pose> user_vel;
    user_vel.setData(tf::Transform(tf::createQuaternionFromYaw(vel->angular.z), tf::Vector3(vel->linear.x, vel->linear.y, 0)));
    user_vel.frame_id_ = robot_base_frame_;
    user_vel.stamp_ = ros::Time();

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, user_vel, drive_cmds);
    Trajectory user_path = tc_->findPath(global_pose, robot_vel, user_vel);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    if(path.cost_ < 0){
      local_plan.clear();
//      publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
      return false;
    }

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    // Fill out the user plan
    for(unsigned int i = 0; i < user_path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      user_path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      user_plan.push_back(pose);
    }

    //publish information to the visualizer
    publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
    publishPlan(user_plan, u_plan_pub_, 0.0, 0.0, 1.0, 0.0);
    return true;
  }

  void SafeTrajectoryPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub, double r, double g, double b, double a){
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = global_frame_;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }

  bool SafeTrajectoryPlannerROS::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    costmap_ros_->resetLayers();
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_trajectory_node");
#if ROS_VERSION_MINIMUM(1, 14, 0) // ROS_MELODIC
  safe_teleop::TFListener tf;
  tf2_ros::TransformListener tf2_listener(tf);
#else
  safe_teleop::TFListener tf(ros::Duration(10));
#endif
  costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf);
  safe_teleop::SafeTrajectoryPlannerROS* planner = new safe_teleop::SafeTrajectoryPlannerROS(&tf, costmap_ros);

  ros::spin();

  delete planner;
  delete costmap_ros;
  return(0);
}
