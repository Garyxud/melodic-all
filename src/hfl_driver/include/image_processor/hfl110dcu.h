/// Copyright 2019 Continental AG
///
/// @file hfl110dcu.h
///
/// @brief This file defines the HFL110DCU image processor class.
///
#ifndef IMAGE_PROCESSOR_HFL110DCU_H
#define IMAGE_PROCESSOR_HFL110DCU_H

#include <string>
#include <vector>
#include <cmath>
#include <angles/angles.h>
#include <arpa/inet.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <hfl_driver/ToFImage.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <base_hfl110dcu.h>
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#define HFL110_MAGIC_NUMBER_16_BIT 0.000762951           // 50 / 2^16

const float NO_RETURN_DISTANCES = NAN;

namespace hfl
{
/// @brief HFL110DCU v1 frame struct
struct PointCloudReturn
{
  uint16_t range;
  uint16_t intensity;
  uint16_t range2;
  uint16_t intensity2;
};

/// @brief HFL110DCU v1 ethernet packet header struct
struct UdpPacketHeader
{
  uint16_t udp_version;
  uint16_t pca_version;
  uint64_t timeStamp;
  uint32_t upd_packet_number;
  uint32_t image_row_number;
};

/// @brief HFL110DCU v1 ethernet extrinsics struct
struct CameraIntrinsics
{
  float_t fx;
  float_t fy;
  float_t ux;
  float_t uy;
  float_t r1;
  float_t r2;
  float_t t1;
  float_t t2;
  float_t r3;
};

/// @brief HFL110DCU v1 ethernet extrinsics struct
struct CameraExtrinsics
{
  float_t intrinsic_yaw;
  float_t intrinsic_pitch;
  float_t extrinsic_yaw;
  float_t extrinsic_pitch;
  float_t extrinsic_roll;
  float_t extrinsic_vertical;
  float_t extrinsic_horizontal;
  float_t extrinsic_distance;
  uint32_t status;
};

/// @brief HFL110DCU v1 ethernet frame struct
struct UdpFrame
{
  UdpPacketHeader header;
  CameraIntrinsics camera_intrinsics;
  CameraExtrinsics camera_extrinsics;
  PointCloudReturn returns[128];
  uint8_t pixel_type[128];
};

///
/// @brief Implements the HFL110DCU camera image parsing and publishing.
///
class HFL110DCU : public BaseHFL110DCU
{
public:
  ///
  /// HFL110DCU image processor constructor.
  ///
  /// @param[in] model camera hfl model
  /// @param[in] version camera version
  /// @param[in] frame_id camera's coordinate frame name
  /// @param[in] node_handler reference to the ros node handler
  ///
  HFL110DCU(std::string model, std::string version,
            std::string frame_id, ros::NodeHandle& node_handler);

  ///
  /// Parse out the packet data into depth and intensity images
  ///
  /// @param[in] starting byte, packet to parse
  ///
  /// @return bool true if successfully parsed packet
  ///
  bool parseFrame(int start_byte, const std::vector<uint8_t>& packet) override;

  ///
  /// Process data frame from udp packets.
  ///
  /// @param[in] data frame data array
  ///
  /// @return bool true if successful
  ///
  bool processFrameData(const std::vector<uint8_t>& data) override;

private:
  /// ROS node handler
  ros::NodeHandle node_handler_;

  /// Received packet bytes from HFL110
  int bytes_received_;

  /// Frame Header message
  std::shared_ptr<std_msgs::Header> frame_header_message_;

  /// TF Header message
  std::shared_ptr<std_msgs::Header> tf_header_message_;

  /// Row and column Counter
  uint8_t row_, col_;

  /// Return counter
  uint8_t expected_packet_ = 0;

  // Camera info manager
  camera_info_manager::CameraInfoManager *camera_info_manager_;

  /// Pointer to depth image
  cv_bridge::CvImagePtr p_image_depth_;

  /// Pointer to 16 bit intensity image
  cv_bridge::CvImagePtr p_image_intensity_;

  /// Depth image publisher
  image_transport::CameraPublisher pub_depth_;

  /// Depth image publisher second return 2
  image_transport::CameraPublisher pub_depth2_;

  /// 16 bit Intensity image publisher
  image_transport::CameraPublisher pub_intensity_;

  /// 16 bit Intensity image publisher return 2
  image_transport::CameraPublisher pub_intensity2_;

  /// Pointer to depth image second return
  cv_bridge::CvImagePtr p_image_depth2_;

  /// Pointer to 16 bit intensity image second return
  cv_bridge::CvImagePtr p_image_intensity2_;
};
}  // namespace hfl
#endif  // IMAGE_PROCESSOR_HFL110DCU_H
