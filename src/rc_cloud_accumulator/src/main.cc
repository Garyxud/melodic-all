#define PCL_NO_PRECOMPILE
#include "cloud_accumulator.h"
#include <ros/ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rc_cloud_accumulator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string output_filename = pnh.param("output_filename", std::string("cloud.pcd"));
  double downsampling_size_display = pnh.param("voxel_grid_size_display", 0.05);//0.0 or less: Off
  double downsampling_size_save = pnh.param("voxel_grid_size_save", 0.01);//0.0 or less: Off
  double min_distance = pnh.param("minimum_distance", 0.0);
  double max_distance = pnh.param("maximum_distance", 5.0);
  bool start_paused = pnh.param("start_paused", false);
  bool keep_high_res = pnh.param("keep_high_resolution", true);

  rc::CloudAccumulator acc(downsampling_size_display, downsampling_size_save,
                           min_distance, max_distance,
                           output_filename, keep_high_res, start_paused);

  ros::Subscriber traj_sub;
  if(keep_high_res)
  {
    traj_sub = nh.subscribe<nav_msgs::Path>("trajectory", 2,
                                            &rc::CloudAccumulator::trajectoryCallback, &acc);
  }

  ros::Subscriber cloud_sub =
    nh.subscribe<pointcloud_t>("stereo/points2", 1, &rc::CloudAccumulator::pointCloudCallback, &acc);

  ros::Subscriber pose_sub =
    nh.subscribe<geometry_msgs::PoseStamped>("pose", 50, &rc::CloudAccumulator::poseCallback, &acc);

  ros::ServiceServer pause_srv =
    pnh.advertiseService("toggle_pause", &rc::CloudAccumulator::togglePause, &acc);

  ros::ServiceServer save_srv =
    pnh.advertiseService("save_cloud", &rc::CloudAccumulator::saveCloud, &acc);

  ros::AsyncSpinner spinner(4);//Async spinner, since the save_cloud service may take long
  spinner.start();
  ros::waitForShutdown();
}
