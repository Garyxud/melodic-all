#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

typedef pcl::visualization::PCLVisualizer visualizer_t;
typedef boost::shared_ptr<visualizer_t> visualizer_ptr_t;
typedef pcl::PointXYZRGB point_t;
typedef pcl::PointCloud<point_t> pointcloud_t;

namespace rc
{

///Displays one cloud using the PCLVisualizer.
///To be used as thread.
class CloudVisualizer {
  public:

  ///Creates the viewer object
  CloudVisualizer(const char* name);

  ///Viewer update loop, will be called when viewer thread is started
  void operator()();

  ///Thread-safe addition/replacement of the cloud
  void addCloudToViewer(const pointcloud_t::Ptr pointcloud);

  ///Thread-safe update to the cloud
  void updateCloudInViewer(const pointcloud_t::Ptr pointcloud);

  ///Signal thread to stop the visualization loop
  void stop();

  private:

  visualizer_ptr_t viewer_;
  bool stop_;
  boost::mutex viewer_mutex_;
};


}//namespace rc
