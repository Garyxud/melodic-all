/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef SAFE_TRAJECTORY_PLANNER_H_
#define SAFE_TRAJECTORY_PLANNER_H_

#include <vector>
#include <math.h>
#include <ros/console.h>
#include <angles/angles.h>

//for creating a local cost grid
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <base_local_planner/world_model.h>

#include <base_local_planner/trajectory.h>

//we'll take in a path as a vector of poses
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <base_local_planner/Position2DInt.h>

//for computing path distance
#include <queue>

//for some datatypes
#include <tf/transform_datatypes.h>

namespace safe_teleop {
  /**
   * @class SafeTrajectoryPlanner
   * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world. 
   */
  class SafeTrajectoryPlanner{
    friend class SafeTrajectoryPlannerTest; //Need this for gtest to work
    public:
      /**
       * @brief  Constructs a trajectory controller
       * @param world_model The WorldModel the trajectory controller uses to check for collisions 
       * @param costmap A reference to the Costmap the controller should use
       * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
       * @param inscribed_radius The radius of the inscribed circle of the robot
       * @param circumscribed_radius The radius of the circumscribed circle of the robot
       * @param acc_lim_x The acceleration limit of the robot in the x direction
       * @param acc_lim_y The acceleration limit of the robot in the y direction
       * @param acc_lim_theta The acceleration limit of the robot in the theta direction
       * @param sim_time The number of seconds to "roll-out" each trajectory
       * @param sim_granularity The distance between simulation points should be small enough that the robot doesn't hit things
       * @param vx_samples The number of trajectories to sample in the x dimension
       * @param vy_samples The number of trajectories to sample in the y dimension
       * @param vtheta_samples The number of trajectories to sample in the theta dimension
       * @param userdist_scale A scaling factor for how close the robot should stay to the user's commanded path
       * @param heading_scale A scaling factor for how close the robot should stay to the user's commanded direction
       * @param occdist_scale A scaling factor for how much the robot should prefer to stay away from obstacles
       * @param heading_lookahead How far the robot should look ahead of itself when differentiating between different rotational velocities
       * @param oscillation_reset_dist The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
       * @param holonomic_robot Set this to true if the robot being controlled can take y velocities and false otherwise
       * @param max_vel_x The maximum x velocity the controller will explore
       * @param min_vel_x The minimum x velocity the controller will explore
       * @param max_vel_y The maximum y velocity the controller will explore
       * @param min_vel_y The minimum y velocity the controller will explore
       * @param max_vel_th The maximum rotational velocity the controller will explore
       * @param min_vel_th The minimum rotational velocity the controller will explore
       * @param holonomic_robot Set to true if robot is holonomic
       * @param dwa Set this to true to use the Dynamic Window Approach, false to use acceleration limits
       */
      SafeTrajectoryPlanner(base_local_planner::WorldModel& world_model,
          const costmap_2d::Costmap2D& costmap, 
          std::vector<geometry_msgs::Point> footprint_spec,
          double inscribed_radius, double circumscribed_radius,
          double acc_lim_x = 1.0, double acc_lim_y = 1.0, double acc_lim_theta = 1.0,
          double sim_time = 1.0, double sim_granularity = 0.025, 
          int vx_samples = 10, int vy_samples = 10, int vtheta_samples = 10,
          double userdist_scale = 0.8, double occdist_scale = 0.2,
          double max_vel_x = 0.5, double min_vel_x = 0.1,
          double max_vel_y = 0.2, double min_vel_y = -0.2,
          double max_vel_th = 1.0, double min_vel_th = -1.0,
          bool holonomic_robot = true, bool dwa = false);

      /**
       * @brief  Destructs a trajectory controller
       */
      ~SafeTrajectoryPlanner();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
       * @param global_pose The current pose of the robot in world space 
       * @param global_vel The current velocity of the robot in world space
       * @param drive_velocities Will be set to velocities to send to the robot base
       * @return The selected path or trajectory
       */
      base_local_planner::Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose> user_vel, tf::Stamped<tf::Pose>& drive_velocities);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
       * @param global_pose The current pose of the robot in world space
       * @param global_vel The current velocity of the robot in world space
       * @param user_vel The velocities to use to construct the path
       * @return The selected path or trajectory
       */
      base_local_planner::Trajectory findPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose> user_vel);

    private:
      /**
       * @brief  Create the trajectories we wish to explore, score them, and return the best option
       * @param x The x position of the robot  
       * @param y The y position of the robot  
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @param dx The desire x velocity
       * @param dy The desired y velocity
       * @param dtheta The desired theta velocity
       * @return 
       */
      base_local_planner::Trajectory createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta,
          double acc_x, double acc_y, double acc_theta, double dx, double dy, double dtheta);

      /**
       * @brief  Generate and score a single trajectory
       * @param x The x position of the robot  
       * @param y The y position of the robot  
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @param impossible_cost The cost value of a cell in the local map grid that is considered impassable
       * @param dx The desire x velocity
       * @param dy The desired y velocity
       * @param dtheta The desired theta velocity
       * @param traj Will be set to the generated trajectory with its associated score 
       */
      void generateTrajectory(double x, double y, double theta, double vx, double vy, 
          double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y,
          double acc_theta, double impossible_cost, double dx, double dy, double dtheta,
          base_local_planner::Trajectory& traj);

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
      double footprintCost(double x_i, double y_i, double theta_i);

      /**
       * @brief  Used to get the cells that make up the footprint of the robot
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @param  fill If true: returns all cells in the footprint of the robot. If false: returns only the cells that make up the outline of the footprint.
       * @return The cells that make up either the outline or entire footprint of the robot depending on fill
       */
      std::vector<base_local_planner::Position2DInt> getFootprintCells(double x_i, double y_i, double theta_i, bool fill);

      /**
       * @brief  Use Bresenham's algorithm to trace a line between two points in a grid
       * @param  x0 The x coordinate of the first point
       * @param  x1 The x coordinate of the second point
       * @param  y0 The y coordinate of the first point
       * @param  y1 The y coordinate of the second point
       * @param  pts Will be filled with the cells that lie on the line in the grid
       */
      void getLineCells(int x0, int x1, int y0, int y1, std::vector<base_local_planner::Position2DInt>& pts);

      /**
       * @brief Fill the outline of a polygon, in this case the robot footprint, in a grid
       * @param footprint The list of cells making up the footprint in the grid, will be modified to include all cells inside the footprint
       */
      void getFillCells(std::vector<base_local_planner::Position2DInt>& footprint);

      base_local_planner::MapGrid map_; ///< @brief The local map grid where we propagate goal and path distance
      const costmap_2d::Costmap2D& costmap_; ///< @brief Provides access to cost map information
      base_local_planner::WorldModel& world_model_; ///< @brief The world model that the controller uses for collision detection

      std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot
      double inscribed_radius_, circumscribed_radius_; ///< @brief The inscribed and circumscribed radii of the robot

      double sim_time_;        ///< @brief The number of seconds each trajectory is "rolled-out"
      double sim_granularity_; ///< @brief The distance between simulation points

      int vx_samples_;     /// < @brief The number of samples we'll take in the x dimension of the control space
      int vy_samples_;     /// < @brief The number of samples we'll take in the y dimension of the control space
      int vtheta_samples_; /// < @brief The number of samples we'll take in the theta dimension of the control space

      double userdist_scale_, occdist_scale_; ///< @brief Scaling factors for the controller's cost function
      double acc_lim_x_, acc_lim_y_, acc_lim_theta_;          ///< @brief The acceleration limits of the robot

      base_local_planner::Trajectory traj_one, traj_two; ///< @brief Used for scoring trajectories

      double max_vel_x_, min_vel_x_, max_vel_y_, min_vel_y_, max_vel_th_, min_vel_th_; ///< @brief Velocity limits for the controller
      
      bool holonomic_robot_;

      bool dwa_;


      /**
       * @brief  Compute x position based on velocity
       * @param  xi The current x position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new x position 
       */
      inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
        return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute y position based on velocity
       * @param  yi The current y position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new y position 
       */
      inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute orientation based on velocity
       * @param  thetai The current orientation
       * @param  vth The current theta velocity
       * @param  dt The timestep to take
       * @return The new orientation
       */
      inline double computeNewThetaPosition(double thetai, double vth, double dt){
        return thetai + vth * dt;
      }

      //compute velocity based on acceleration
      /**
       * @brief  Compute velocity based on acceleration
       * @param vg The desired velocity, what we're accelerating up to 
       * @param vi The current velocity
       * @param a_max An acceleration limit
       * @param  dt The timestep to take
       * @return The new velocity
       */
      inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
        if(vg >= 0)
          return std::min(vg, vi + a_max * dt);
        return std::max(vg, vi - a_max * dt);
      }

      double lineCost(int x0, int x1, int y0, int y1);
      double pointCost(int x, int y);
//      double headingDiff(int cell_x, int cell_y, double x, double y, double heading);
  };
};

#endif
