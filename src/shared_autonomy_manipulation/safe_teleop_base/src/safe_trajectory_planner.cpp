#include <safe_teleop_base/safe_trajectory_planner.h>

using namespace std;
using namespace costmap_2d;
using namespace base_local_planner;

namespace safe_teleop {
SafeTrajectoryPlanner::SafeTrajectoryPlanner(WorldModel& world_model,
    const Costmap2D& costmap,
    std::vector<geometry_msgs::Point> footprint_spec,
    double inscribed_radius, double circumscribed_radius,
    double acc_lim_x, double acc_lim_y, double acc_lim_theta,
    double sim_time, double sim_granularity,
    int vx_samples, int vy_samples, int vtheta_samples,
    double userdist_scale, double occdist_scale,
    double max_vel_x, double min_vel_x,
    double max_vel_y, double min_vel_y,
    double max_vel_th, double min_vel_th,
    bool holonomic_robot, bool dwa)
: map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), costmap_(costmap),
  world_model_(world_model), footprint_spec_(footprint_spec),
  inscribed_radius_(inscribed_radius), circumscribed_radius_(circumscribed_radius),
  sim_time_(sim_time), sim_granularity_(sim_granularity),
  vx_samples_(vx_samples), vy_samples_(vy_samples), vtheta_samples_(vtheta_samples),
  userdist_scale_(userdist_scale), occdist_scale_(occdist_scale),
  acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
  max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
  max_vel_y_(max_vel_y), min_vel_y_(min_vel_y),
  max_vel_th_(max_vel_th), min_vel_th_(min_vel_th),
  holonomic_robot_(holonomic_robot), dwa_(dwa)
{
}

SafeTrajectoryPlanner::~SafeTrajectoryPlanner(){}

//create and score a trajectory given the current pose of the robot and selected velocities
void SafeTrajectoryPlanner::generateTrajectory(
    double x, double y, double theta,
    double vx, double vy, double vtheta,
    double vx_samp, double vy_samp, double vtheta_samp,
    double acc_x, double acc_y, double acc_theta,
    double impossible_cost,
    double dx, double dy, double dtheta,
    Trajectory& traj){
  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i, vy_i, vtheta_i;

  vx_i = vx;
  vy_i = vy;
  vtheta_i = vtheta;

  //compute the magnitude of the velocities
  //    double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);

  //compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = int(sim_time_ / sim_granularity_ + 0.5);

  double dt = sim_time_ / num_steps;
  double time = 0.0;

  //create a potential trajectory
  traj.resetPoints();
  traj.xv_ = vx_samp;
  traj.yv_ = vy_samp;
  traj.thetav_ = vtheta_samp;
  traj.cost_ = -1.0;

  if(num_steps == 0)
    return;

  //initialize the costs for the trajectory
  double user_dist = 0.0;
  double occ_cost = 0.0;

  for(int i = 0; i < num_steps; ++i){
    //get map coordinates of a point
    unsigned int cell_x, cell_y;

    //we don't want a path that goes off the know map
    if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
      traj.cost_ = -1.0;
      return;
    }

    //check the point on the trajectory for legality
    double footprint_cost = footprintCost(x_i, y_i, theta_i);

    //if the footprint hits an obstacle this trajectory is invalid
    if(footprint_cost < 0){
      traj.cost_ = -1.0;
      return;
    }

    occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

    time += dt;

    //the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    //calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    //calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

  }

  user_dist = sqrt((vx_samp - dx) * (vx_samp - dx) +
      (vy_samp - dy) * (vy_samp - dy) +
      (vtheta_samp - dtheta) * (vtheta_samp - dtheta));

  traj.cost_ = user_dist * userdist_scale_ + occdist_scale_ * occ_cost;
}

//calculate the cost of a ray-traced line
double SafeTrajectoryPlanner::lineCost(int x0, int x1,
    int y0, int y1){
  //Bresenham Ray-Tracing
  int deltax = abs(x1 - x0);        // The difference between the x's
  int deltay = abs(y1 - y0);        // The difference between the y's
  int x = x0;                       // Start x off at the first pixel
  int y = y0;                       // Start y off at the first pixel

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;

  double line_cost = 0.0;
  double point_cost = -1.0;

  if (x1 >= x0)                 // The x-values are increasing
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          // The x-values are decreasing
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y1 >= y0)                 // The y-values are increasing
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          // The y-values are decreasing
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         // There is at least one x-value for every y-value
  {
    xinc1 = 0;                  // Don't change the x when numerator >= denominator
    yinc2 = 0;                  // Don't change the y for every iteration
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         // There are more x-values than y-values
  }
  else                          // There is at least one y-value for every x-value
  {
    xinc2 = 0;                  // Don't change the x for every iteration
    yinc1 = 0;                  // Don't change the y when numerator >= denominator
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         // There are more y-values than x-values
  }

  for (int curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    point_cost = pointCost(x, y); //Score the current point

    if(point_cost < 0)
      return -1;

    if(line_cost < point_cost)
      line_cost = point_cost;

    num += numadd;              // Increase the numerator by the top of the fraction
    if (num >= den)             // Check if numerator >= denominator
    {
      num -= den;               // Calculate the new numerator value
      x += xinc1;               // Change the x as appropriate
      y += yinc1;               // Change the y as appropriate
    }
    x += xinc2;                 // Change the x as appropriate
    y += yinc2;                 // Change the y as appropriate
  }

  return line_cost;
}

double SafeTrajectoryPlanner::pointCost(int x, int y){
  unsigned char cost = costmap_.getCost(x, y);
  //if the cell is in an obstacle the path is invalid
  if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
    return -1;
  }

  return cost;
}

//create the trajectories we wish to score
Trajectory SafeTrajectoryPlanner::createTrajectories(double x, double y, double theta,
    double vx, double vy, double vtheta,
    double acc_x, double acc_y, double acc_theta,
    double dx, double dy, double dtheta){
  //compute feasible velocity limits in robot space
  double max_vel_x, max_vel_y, max_vel_theta;
  double min_vel_x, min_vel_y, min_vel_theta;

  //should we use the dynamic window approach?
  if(dwa_){
    max_vel_x = min(max_vel_x_, vx + acc_x * .1);
    min_vel_x = max(min_vel_x_, vx - acc_x * .1);

    max_vel_y = min(max_vel_y_, vy + acc_y * .1);
    min_vel_y = max(min_vel_y_, vy - acc_y * .1);

    max_vel_theta = min(max_vel_th_, vtheta + acc_theta * .1);
    min_vel_theta = max(min_vel_th_, vtheta - acc_theta * .1);
  } else {
    max_vel_x = min(max_vel_x_, vx + acc_x * sim_time_);
    min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

    max_vel_y = min(max_vel_y_, vy + acc_y * sim_time_);
    min_vel_y = max(min_vel_y_, vy - acc_y * sim_time_);

    max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
    min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
  }

  //keep track of the best trajectory seen so far
  Trajectory* best_traj = &traj_one;
  best_traj->cost_ = -1.0;

  Trajectory* comp_traj = &traj_two;
  comp_traj->cost_ = -1.0;

  Trajectory* swap = NULL;

  //any cell with a cost greater than the size of the map is impossible
  double impossible_cost = map_.obstacleCosts();

  // first sample the user suggested trajectory
  generateTrajectory(x, y, theta, vx, vy, vtheta, dx, dy, dtheta, acc_x, acc_y, acc_theta, impossible_cost, dx, dy, dtheta, *comp_traj);

  //if the new trajectory is better... let's take it
  if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
  }

  //we want to sample the velocity space regularly
  double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
  double dvy = (max_vel_y - min_vel_y) / (vy_samples_ - 1);
  double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

  double vx_samp = 0.0;
  double vy_samp = 0.0;
  double vtheta_samp = 0.0;

  if (holonomic_robot_) {
    //loop through all x velocities
    vx_samp = min_vel_x;
    for(int i = 0; i < vx_samples_; ++i){
      vy_samp = 0;
      vtheta_samp = 0;
      //first sample the straight trajectory
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, dx, dy, dtheta, *comp_traj);

      //if the new trajectory is better... let's take it
      if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
        swap = best_traj;
        best_traj = comp_traj;
        comp_traj = swap;
      }

      vy_samp = min_vel_y;
      for (int k = 0; k < vy_samples_; k++) {
        vtheta_samp = 0;
        //first sample the trajectory with no rotation
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, dx, dy, dtheta, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }


        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories
        for(int j = 0; j < vtheta_samples_ - 1; ++j){
          generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, dx, dy, dtheta, *comp_traj);

          //if the new trajectory is better... let's take it
          if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
          }
          vtheta_samp += dvtheta;
        }
        vy_samp += dvy;
      }
      vx_samp += dvx;
    }
  } else {
    vy_samp = 0.0;
    //loop through all x velocities
    vx_samp = min_vel_x;
    for(int i = 0; i < vx_samples_; ++i){
      vtheta_samp = 0;
      //first sample the straight trajectory
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, dx, dy, dtheta, *comp_traj);

      //if the new trajectory is better... let's take it
      if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
        swap = best_traj;
        best_traj = comp_traj;
        comp_traj = swap;
      }

      vtheta_samp = min_vel_theta;
      //next sample all theta trajectories
      for(int j = 0; j < vtheta_samples_ - 1; ++j){
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, dx, dy, dtheta, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
        vtheta_samp += dvtheta;

      }
      vx_samp += dvx;
    }
  }



  return *best_traj;
}

//given the current state of the robot, find a good trajectory
Trajectory SafeTrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
    tf::Stamped<tf::Pose> user_vel, tf::Stamped<tf::Pose>& drive_velocities){

  double yaw = tf::getYaw(global_pose.getRotation());
  double vel_yaw = tf::getYaw(global_vel.getRotation());
  double dyaw = tf::getYaw(user_vel.getRotation());

  double x = global_pose.getOrigin().getX();
  double y = global_pose.getOrigin().getY();
  double theta = yaw;

  double vx = global_vel.getOrigin().getX();
  double vy = global_vel.getOrigin().getY();
  double vtheta = vel_yaw;

  double dx = user_vel.getOrigin().getX();
  double dy = user_vel.getOrigin().getY();
  double dtheta = dyaw;

  //reset the map for new operations
  map_.resetPathDist();

  //temporarily remove obstacles that are within the footprint of the robot
  vector<base_local_planner::Position2DInt> footprint_list = getFootprintCells(x, y, theta, true);

  //mark cells within the initial footprint of the robot
  for(unsigned int i = 0; i < footprint_list.size(); ++i){
    map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
  }

  //rollout trajectories and find the minimum cost one
  Trajectory best = createTrajectories(x, y, theta, vx, vy, vtheta, acc_lim_x_, acc_lim_y_, acc_lim_theta_, dx, dy, dtheta);
  ROS_DEBUG("Trajectories created");

  if(best.cost_ < 0) {
    drive_velocities.setIdentity();
    ROS_INFO("No safe trajectory found");
  }
  else {
    tf::Vector3 start(best.xv_, best.yv_, 0);
    drive_velocities.setOrigin(start);
    tf::Matrix3x3 matrix;
    matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
    drive_velocities.setBasis(matrix);
  }

  return best;
}

Trajectory SafeTrajectoryPlanner::findPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
    tf::Stamped<tf::Pose> user_vel){

  double yaw = tf::getYaw(global_pose.getRotation());
  double vel_yaw = tf::getYaw(global_vel.getRotation());
  double dyaw = tf::getYaw(user_vel.getRotation());

  double x = global_pose.getOrigin().getX();
  double y = global_pose.getOrigin().getY();
  double theta = yaw;

  double vx = global_vel.getOrigin().getX();
  double vy = global_vel.getOrigin().getY();
  double vtheta = vel_yaw;

  double dx = user_vel.getOrigin().getX();
  double dy = user_vel.getOrigin().getY();
  double dtheta = dyaw;

  //reset the map for new operations
  map_.resetPathDist();

  //temporarily remove obstacles that are within the footprint of the robot
  vector<base_local_planner::Position2DInt> footprint_list = getFootprintCells(x, y, theta, true);

  //mark cells within the initial footprint of the robot
  for(unsigned int i = 0; i < footprint_list.size(); ++i){
    map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
  }

  Trajectory traj;
  double impossible_cost = map_.obstacleCosts();
  generateTrajectory(x, y, theta, vx, vy, vtheta, dx, dy, dtheta,
      acc_lim_x_, acc_lim_y_, acc_lim_theta_, impossible_cost, dx, dy, dtheta, traj);

  return traj;
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double SafeTrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
  //build the oriented footprint
  double cos_th = cos(theta_i);
  double sin_th = sin(theta_i);
  vector<geometry_msgs::Point> oriented_footprint;
  for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
    geometry_msgs::Point new_pt;
    new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
    new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  geometry_msgs::Point robot_position;
  robot_position.x = x_i;
  robot_position.y = y_i;

  //check if the footprint is legal
  double footprint_cost = world_model_.footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);

  return footprint_cost;
}

void SafeTrajectoryPlanner::getLineCells(int x0, int x1, int y0, int y1, vector<base_local_planner::Position2DInt>& pts){
  //Bresenham Ray-Tracing
  int deltax = abs(x1 - x0);        // The difference between the x's
  int deltay = abs(y1 - y0);        // The difference between the y's
  int x = x0;                       // Start x off at the first pixel
  int y = y0;                       // Start y off at the first pixel

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;

  base_local_planner::Position2DInt pt;

  if (x1 >= x0)                 // The x-values are increasing
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          // The x-values are decreasing
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y1 >= y0)                 // The y-values are increasing
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          // The y-values are decreasing
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         // There is at least one x-value for every y-value
  {
    xinc1 = 0;                  // Don't change the x when numerator >= denominator
    yinc2 = 0;                  // Don't change the y for every iteration
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         // There are more x-values than y-values
  }
  else                          // There is at least one y-value for every x-value
  {
    xinc2 = 0;                  // Don't change the x for every iteration
    yinc1 = 0;                  // Don't change the y when numerator >= denominator
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         // There are more y-values than x-values
  }

  for (int curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    pt.x = x;      //Draw the current pixel
    pt.y = y;
    pts.push_back(pt);

    num += numadd;              // Increase the numerator by the top of the fraction
    if (num >= den)             // Check if numerator >= denominator
    {
      num -= den;               // Calculate the new numerator value
      x += xinc1;               // Change the x as appropriate
      y += yinc1;               // Change the y as appropriate
    }
    x += xinc2;                 // Change the x as appropriate
    y += yinc2;                 // Change the y as appropriate
  }
}

//get the cellsof a footprint at a given position
vector<base_local_planner::Position2DInt> SafeTrajectoryPlanner::getFootprintCells(double x_i, double y_i, double theta_i, bool fill){
  vector<base_local_planner::Position2DInt> footprint_cells;

  //if we have no footprint... do nothing
  if(footprint_spec_.size() <= 1){
    unsigned int mx, my;
    if(costmap_.worldToMap(x_i, y_i, mx, my)){
      Position2DInt center;
      center.x = mx;
      center.y = my;
      footprint_cells.push_back(center);
    }
    return footprint_cells;
  }

  //pre-compute cos and sin values
  double cos_th = cos(theta_i);
  double sin_th = sin(theta_i);
  double new_x, new_y;
  unsigned int x0, y0, x1, y1;
  unsigned int last_index = footprint_spec_.size() - 1;

  for(unsigned int i = 0; i < last_index; ++i){
    //find the cell coordinates of the first segment point
    new_x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
    new_y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x0, y0))
      return footprint_cells;

    //find the cell coordinates of the second segment point
    new_x = x_i + (footprint_spec_[i + 1].x * cos_th - footprint_spec_[i + 1].y * sin_th);
    new_y = y_i + (footprint_spec_[i + 1].x * sin_th + footprint_spec_[i + 1].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x1, y1))
      return footprint_cells;

    getLineCells(x0, x1, y0, y1, footprint_cells);
  }

  //we need to close the loop, so we also have to raytrace from the last pt to first pt
  new_x = x_i + (footprint_spec_[last_index].x * cos_th - footprint_spec_[last_index].y * sin_th);
  new_y = y_i + (footprint_spec_[last_index].x * sin_th + footprint_spec_[last_index].y * cos_th);
  if(!costmap_.worldToMap(new_x, new_y, x0, y0))
    return footprint_cells;

  new_x = x_i + (footprint_spec_[0].x * cos_th - footprint_spec_[0].y * sin_th);
  new_y = y_i + (footprint_spec_[0].x * sin_th + footprint_spec_[0].y * cos_th);
  if(!costmap_.worldToMap(new_x, new_y, x1, y1))
    return footprint_cells;

  getLineCells(x0, x1, y0, y1, footprint_cells);

  if(fill)
    getFillCells(footprint_cells);

  return footprint_cells;
}

void SafeTrajectoryPlanner::getFillCells(vector<base_local_planner::Position2DInt>& footprint){
  //quick bubble sort to sort pts by x
  base_local_planner::Position2DInt swap, pt;
  unsigned int i = 0;
  while(i < footprint.size() - 1){
    if(footprint[i].x > footprint[i + 1].x){
      swap = footprint[i];
      footprint[i] = footprint[i + 1];
      footprint[i + 1] = swap;
      if(i > 0)
        --i;
    }
    else
      ++i;
  }

  i = 0;
  base_local_planner::Position2DInt min_pt;
  base_local_planner::Position2DInt max_pt;
  unsigned int min_x = footprint[0].x;
  unsigned int max_x = footprint[footprint.size() -1].x;
  //walk through each column and mark cells inside the footprint
  for(unsigned int x = min_x; x <= max_x; ++x){
    if(i >= footprint.size() - 1)
      break;

    if(footprint[i].y < footprint[i + 1].y){
      min_pt = footprint[i];
      max_pt = footprint[i + 1];
    }
    else{
      min_pt = footprint[i + 1];
      max_pt = footprint[i];
    }

    i += 2;
    while(i < footprint.size() && footprint[i].x == x){
      if(footprint[i].y < min_pt.y)
        min_pt = footprint[i];
      else if(footprint[i].y > max_pt.y)
        max_pt = footprint[i];
      ++i;
    }

    //loop though cells in the column
    for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
      pt.x = x;
      pt.y = y;
      footprint.push_back(pt);
    }
  }
}

};


