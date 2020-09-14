#define PCL_NO_PRECOMPILE
#include "cloud_visualizer.h"
#include <tf2_ros/buffer.h>
#include <deque>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB point_t;
typedef pcl::PointCloud<point_t> pointcloud_t;

namespace rc
{

/**
 * Demonstration for how to create a registered point cloud map using the
 * rc_visard.
 */
class CloudAccumulator
{

  public:
  CloudAccumulator(double voxelgrid_size_display,
                   double voxelgrid_size_save,
                   double min_distance,
                   double max_distance,
                   std::string output_filename,
                   bool keep_high_res,
                   bool start_paused);

  /**
   * For each cloud
   * - Filter by distance along the optical axis
   * - Store filtered cloud for later saving (if keep_high_res_)
   * - Transform to world coordinates according to live pose
   * - Merge with previous clouds
   * - Apply "display" voxel grid filter on merged cloud
   * - Update viewer with the result
   */
  void pointCloudCallback(const pointcloud_t::ConstPtr& pointcloud);

  /** Replace all saved camera poses by those in \p path */
  void trajectoryCallback(const nav_msgs::PathConstPtr& path);

  /** Saves camera pose */
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& current_pose);

  /**
   * For all saved clouds
   * - Merge
   * - Voxel-grid filter
   * - Display
   */
  pointcloud_t::Ptr rebuildCloud();

  /**
   * - rebuild cloud (if keep_high_res is set) and update viewer
   * - Save as .pcd to disk
   */
  bool saveCloud(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  /** Toggle whether to ignore incoming data */
  bool togglePause(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  private:
  Eigen::Affine3f lookupTransform(double timestamp);

  //Members
  std::deque<pointcloud_t::Ptr> clouds_;///< Storage for later corraection and saving
  boost::mutex tf_buffer_mutex_;
  tf2_ros::Buffer tf_buffer_;      ///< Stores the history of poses and allows to interpolate
  size_t num_poses_;               ///< Keep track of pose count (to show a good error message)
  boost::mutex display_cloud_mutex_;
  pointcloud_t::Ptr display_cloud_;///< Cloud that is displayed
  double voxelgrid_size_display_;  ///< Filter size for the display cloud
  double voxelgrid_size_save_;     ///< Filter size for the saved cloud
  double min_distance_;            ///< Discard points closer than this
  double max_distance_;            ///< Discard points farther than this
  std::string output_filename_;    ///< Where to save the cloud
  bool keep_high_res_;             ///< If false, do not store clouds. On save, save display cloud
  bool pause_;                     ///< Whether to ignore input data
  CloudVisualizer viewer_;
  boost::thread viewer_thread_;
  ros::Time last_pose_stamp_;
};

}//namespace rc
