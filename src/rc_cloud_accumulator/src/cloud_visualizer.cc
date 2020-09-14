#include "cloud_visualizer.h"
#include <ros/ros.h>

namespace rc
{

CloudVisualizer::CloudVisualizer(const char* name) : viewer_(new visualizer_t(name))
{
  viewer_->setBackgroundColor(0.2, 0.2, 0.2);//dark gray
  viewer_->addCoordinateSystem(0.1);//make a small coordinate system where the world pose is
  viewer_->initCameraParameters();
  viewer_->setCameraPosition(-1,0,0,  //1m behind the origin (x is the viewing direction).
                              0,0,0,  //look towards the origin.
                              0,0,1); //Z is up
}

void CloudVisualizer::stop(){ stop_ = true; }

void CloudVisualizer::operator()()
{
  while(!viewer_->wasStopped() && ros::ok())
  {
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    if(!ros::ok()) break;
    boost::mutex::scoped_lock guard(viewer_mutex_);
    viewer_->spinOnce();
  }
}

void CloudVisualizer::addCloudToViewer(const pointcloud_t::Ptr pointcloud)
{
  boost::mutex::scoped_lock guard(viewer_mutex_);
  viewer_->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerRGBField<point_t> rgb(pointcloud);
  viewer_->addPointCloud<point_t>(pointcloud, rgb);
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
}

void CloudVisualizer::updateCloudInViewer(const pointcloud_t::Ptr pointcloud)
{
  boost::mutex::scoped_lock guard(viewer_mutex_);
  pcl::visualization::PointCloudColorHandlerRGBField<point_t> rgb(pointcloud);
  viewer_->updatePointCloud<point_t>(pointcloud, rgb);
}

}//namespace rc

