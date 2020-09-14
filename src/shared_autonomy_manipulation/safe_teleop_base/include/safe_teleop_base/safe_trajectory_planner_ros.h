/*
 * safe_trajectory_planner.h
 *
 *  Created on: Mar 25, 2010
 *      Author: duhadway
 */

#ifndef SAFE_TRAJECTORY_PLANNER_ROS_H_
#define SAFE_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <safe_teleop_base/safe_trajectory_planner.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>


namespace safe_teleop {

#if ROS_VERSION_MINIMUM(1, 14, 0) // ROS_MELODIC
typedef tf2_ros::Buffer TFListener;
#else
typedef tf::TransformListener TFListener;
#endif

/**
   * @class SafeTrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class SafeTrajectoryPlannerROS {
    public:
      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      SafeTrajectoryPlannerROS(TFListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~SafeTrajectoryPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param vel Velocity received from user
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(const geometry_msgs::Twist::ConstPtr& vel, geometry_msgs::Twist& cmd_vel);

    private:
      /**
       * @brief  Check whether the robot is stopped or not
       * @return True if the robot is stopped, false otherwise
       */
      bool stopped();

      /**
       * @brief  Compute the distance between two points
       * @param x1 The first x point
       * @param y1 The first y point
       * @param x2 The second x point
       * @param y2 The second y point
       */
      double distance(double x1, double y1, double x2, double y2);

      /**
       * @brief  Transforms the global plan of the robot from the planner frame to the local frame
       * @param transformed_plan Populated with the transformed plan
       */
      bool transformGlobalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      /**
       * @brief  Publish a plan for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub, double r, double g, double b, double a);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void cmdCallback(const geometry_msgs::Twist::ConstPtr& vel);

      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      std::vector<double> loadYVels(ros::NodeHandle node);

      ros::NodeHandle nh_;

      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
      SafeTrajectoryPlanner* tc_; ///< @brief The trajectory controller
      costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
      costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will use
      TFListener* tf_; ///< @brief Used for transforming point clouds
      std::string global_frame_; ///< @brief The frame in which the controller will run
      double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
      nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
      std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
      double rot_stopped_velocity_, trans_stopped_velocity_;
      double inscribed_radius_, circumscribed_radius_;
      ros::Publisher l_plan_pub_;
      ros::Publisher u_plan_pub_;
      ros::Subscriber odom_sub_, user_sub_;

      ros::Publisher cmd_pub_;
      ros::Subscriber cmd_sub_;

      ros::ServiceServer clear_costmaps_srv_;

      boost::recursive_mutex odom_lock_;
      double max_vel_th_, min_vel_th_;
      bool safe_backwards_;
  };

}


#endif /* SAFE_TRAJECTORY_PLANNER_ROS_H_ */
