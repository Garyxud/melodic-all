/// Copyright 2019 Continental AG
///
/// @file hfl110dcu.cpp
///
/// @brief This file implements the hfl110dcu image processor class methods
///
#include "image_processor/hfl110dcu.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <cmath>

namespace hfl
{
HFL110DCU::HFL110DCU(std::string model, std::string version,
                     std::string frame_id, ros::NodeHandle& node_handler)
  : node_handler_(node_handler)
{
  // Set model and version
  model_ = model;
  version_ = version;

  // Initialize header messages
  frame_header_message_.reset(new std_msgs::Header());

  ros::NodeHandle image_depth_nh(node_handler_, "depth");
  ros::NodeHandle image_intensity_16b_nh(node_handler_, "intensity");
  ros::NodeHandle image_depth2_nh(node_handler_, "depth2");
  ros::NodeHandle image_intensity2_16b_nh(node_handler_, "intensity2");
  ros::NodeHandle image_intensity_8b_nh(node_handler_, "intensity8");

  image_transport::ImageTransport it_depth(image_depth_nh);
  image_transport::ImageTransport it_depth2(image_depth2_nh);
  image_transport::ImageTransport it_intensity_16b(image_intensity_16b_nh);
  image_transport::ImageTransport it_intensity2_16b(image_intensity2_16b_nh);
  image_transport::ImageTransport it_intensity_8b(image_intensity_8b_nh);

  // Initialize publishers
  pub_depth_ = it_depth.advertiseCamera("image_raw", 100);
  pub_intensity_ = it_intensity_16b.advertiseCamera("image_raw", 100);
  pub_depth2_ = it_depth2.advertiseCamera("image_raw", 100);
  pub_intensity2_ = it_intensity2_16b.advertiseCamera("image_raw", 100);

  std::string default_calib_file = "~/.ros/camera_info/default.yaml";

  // Check camera info manager
  camera_info_manager_ =
    new camera_info_manager::CameraInfoManager(image_intensity_16b_nh, frame_id);

  // Initialize Frame Message Header
  frame_header_message_->frame_id = frame_id;
  frame_header_message_->seq = -1;
}

bool HFL110DCU::parseFrame(int start_byte, const std::vector<uint8_t>& packet)
{
  int byte_offset = 0;

  float range_1, range_2 = 0;
  uint16_t intensity_1, intensity_2 = 0;

  // Build up range and intensity images
  for (col_ = 0; col_ < 128; col_ += 1)
  {
    byte_offset = start_byte + (col_ * 4);
    // Populate range images
    range_1 = float(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset]))) / 256.0;
    range_2 = float(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset + 2]))) / 256.0;

    // If range is larger than 49m, set it to NAN
    if (range_1 > 49.0)
      range_1 = NAN;

    if (range_2 > 49.0)
      range_2 = NAN;

    p_image_depth_->image.at<float>(cv::Point(col_, row_)) = range_1;
    p_image_depth2_->image.at<float>(cv::Point(col_, row_)) = range_2;

    // Byte offset for intensity
    // Intensity Data follows Full Row of Depth Data (128 * 2 returns * 2bytes each)
    byte_offset = start_byte + 512 + (col_ * 4);

    // Populate intensity images
    intensity_1 = uint16_t(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset])));
    intensity_2 = uint16_t(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset + 2])));
    p_image_intensity_->image.at<uint16_t>(cv::Point(col_, row_)) = intensity_1;
    p_image_intensity2_->image.at<uint16_t>(cv::Point(col_, row_)) = intensity_2;
  }

  return true;
}

bool HFL110DCU::processFrameData(const std::vector<uint8_t>& frame_data)
{
  if (version_ == "v1")
  {
    int size = frame_data.size();

    // identify packet by fragmentation offset
    row_ = 31 - big_to_native(*reinterpret_cast<const uint32_t*>(&frame_data[16]));
    int frame_num = big_to_native(*reinterpret_cast<const uint32_t*>(&frame_data[12]));

    // Check packet offset continuity
    if ( row_ != expected_packet_)
    {
      ROS_ERROR("Unexpected packet (dropped packet?) expecting: %i, received:  %i",
              expected_packet_, row_);
      expected_packet_ = 31;
      return false;
    }

    // First frame packet, reset frame data
    if (row_ == 31)
    {
      // Set header messages
      frame_header_message_->stamp = ros::Time::now();

      // Reset image pointers
      p_image_depth_.reset(new cv_bridge::CvImage);
      p_image_depth_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      p_image_depth_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_32FC1);

      p_image_intensity_.reset(new cv_bridge::CvImage);
      p_image_intensity_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      p_image_intensity_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_16UC1);

      p_image_depth2_.reset(new cv_bridge::CvImage);
      p_image_depth2_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      p_image_depth2_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_32FC1);

      p_image_intensity2_.reset(new cv_bridge::CvImage);
      p_image_intensity2_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      p_image_intensity2_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_16UC1);

      // Get intrinsic and extrinsic calibration parameters
      // CameraIntrinsics * camera_intrinsics;
      float fx = *reinterpret_cast<const float*>(&frame_data[20]);
      ROS_INFO_ONCE("fx: %.4f", fx);
      float fy = *reinterpret_cast<const float*>(&frame_data[24]);
      ROS_INFO_ONCE("fy: %.4f", fy);
      float ux = *reinterpret_cast<const float*>(&frame_data[28]);
      ROS_INFO_ONCE("ux: %.4f", ux);
      float uy = *reinterpret_cast<const float*>(&frame_data[32]);
      ROS_INFO_ONCE("uy: %.4f", uy);
      float r1 = *reinterpret_cast<const float*>(&frame_data[36]);
      ROS_INFO_ONCE("r1: %.4f", r1);
      float r2 = *reinterpret_cast<const float*>(&frame_data[40]);
      ROS_INFO_ONCE("r2: %.4f", r2);
      float t1 = *reinterpret_cast<const float*>(&frame_data[44]);
      ROS_INFO_ONCE("t1: %.4f", t1);
      float t2 = *reinterpret_cast<const float*>(&frame_data[48]);
      ROS_INFO_ONCE("t2: %.4f", t2);
      float r3 = *reinterpret_cast<const float*>(&frame_data[52]);
      ROS_INFO_ONCE("r3: %.4f", r3);

      float intrinsic_yaw = *reinterpret_cast<const float*>(&frame_data[56]);
      float intrinsic_pitch = *reinterpret_cast<const float*>(&frame_data[60]);
      float extrinsic_yaw = *reinterpret_cast<const float*>(&frame_data[64]);
      float extrinsic_pitch = *reinterpret_cast<const float*>(&frame_data[68]);
      float extrinsic_roll = *reinterpret_cast<const float*>(&frame_data[72]);
      float extrinsic_vertical = *reinterpret_cast<const float*>(&frame_data[76]);
      float extrinsic_horizontal = *reinterpret_cast<const float*>(&frame_data[80]);
      float extrinsic_distance = *reinterpret_cast<const float*>(&frame_data[84]);

      // check camera info manager
      if (camera_info_manager_ != NULL)
      {
        auto ci = camera_info_manager_->getCameraInfo();
        // set default values
        ci.distortion_model = "plumb_bob";
        ci.height = FRAME_ROWS;
        ci.width = FRAME_COLUMNS;

        if (ci.K[0] != fx)
        {
          ROS_WARN("Initialized intrinsics do not match those received from sensor");
          ROS_WARN("Setting intrinsics to values received from sensor");

          ci.D.resize(5);
          ci.D[0] = r1;
          ci.D[1] = r2;
          ci.D[2] = t1;
          ci.D[3] = t2;
          ci.D[4] = r3;

          ci.K[0] = fx;
          ci.K[2] = ux;
          ci.K[4] = fy;
          ci.K[5] = uy;

          ci.P[0] = fx;
          ci.P[2] = ux;
          ci.P[4] = fy;
          ci.P[5] = uy;
          /*
          ROS_WARN("  Distorion Matrix:");
          ROS_WARN("    r1: %f", ci.D[0]);
          ROS_WARN("    r2: %f", ci.D[1]);
          ROS_WARN("    t1: %f", ci.D[2]);
          ROS_WARN("    t2: %f", ci.D[3]);
          ROS_WARN("    r3: %f", ci.D[4]);

          ROS_WARN("  Intrinsic Matrix:");
          ROS_WARN("    fx: %f", ci.K[0]);
          ROS_WARN("    fy: %f", ci.K[4]);
          ROS_WARN("    ux: %f", ci.K[2]);
          ROS_WARN("    uy: %f", ci.K[5]);

          ROS_WARN("  Projection Matrix:");
          ROS_WARN("    fx': %f", ci.P[0]);
          ROS_WARN("    fy': %f", ci.P[4]);
          ROS_WARN("    ux': %f", ci.P[2]);
          ROS_WARN("    uy': %f", ci.P[5]);
          */
          camera_info_manager_->setCameraInfo(ci);
        }
      }
    }

    // Parse image data
    parseFrame(92, frame_data);

    // Last frame packet, pulish frame data
    if (row_ == 0)
    {
      // Get camera info
      auto ci = camera_info_manager_->getCameraInfo();
      sensor_msgs::CameraInfoPtr flash_cam_info(new sensor_msgs::CameraInfo(ci));

      flash_cam_info->header = *frame_header_message_;

      p_image_depth_->header = *frame_header_message_;
      p_image_intensity_->header = *frame_header_message_;
      pub_depth_.publish(p_image_depth_->toImageMsg(), flash_cam_info);
      pub_intensity_.publish(p_image_intensity_->toImageMsg(), flash_cam_info);

      p_image_depth2_->header = *frame_header_message_;
      p_image_intensity2_->header = *frame_header_message_;
      pub_depth2_.publish(p_image_depth2_->toImageMsg(), flash_cam_info);
      pub_intensity2_.publish(p_image_intensity2_->toImageMsg(), flash_cam_info);

      // publish transform
      // sensor_to_vehicle_tf_.header = *tf_header_message_;
      // tf_broadcaster.sendTransform(sensor_to_vehicle_tf_);
    }
    expected_packet_ = (expected_packet_ > 0)? expected_packet_ - 1: 31;
  }
  return true;
}
}  // namespace hfl
