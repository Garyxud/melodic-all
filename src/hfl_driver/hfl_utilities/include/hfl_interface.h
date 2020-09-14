/// Copyright 2019 Continental AG
///
/// @file hfl_interface.h
///
/// @brief This file defines the HFL camera's interface class.
///

#ifndef HFL_INTERFACE_H
#define HFL_INTERFACE_H

#ifdef _WIN32
#include <winsock2.h>
#elif __linux__
#include <arpa/inet.h>  // ntohl()
#endif

#include <string>
#include <vector>

#include <hfl_configs.h>
#include <hfl_frame.h>

namespace hfl
{

static inline uint32_t big_to_native(uint32_t x)
{
  return ntohl(x);
}

static inline uint16_t big_to_native(uint16_t x)
{
  return ntohs(x);
}

static inline uint8_t big_to_native(uint8_t x)
{
  return x;
}

/// UDP ports types
enum udp_port_types
{
  frame_data,
  object_data,
  lut_data
};

/// Number of Bits
enum num_bits
{
  eight_bit = 0,
  ten_bit,
  twelve_bit,
  fourteen_bit
};

///
/// @brief Base class for all of the HFL cameras
///
class HflInterface
{
protected:
  /// Current camera model
  std::string model_;

  /// Current camera model
  std::string version_;

  /// Camera's IP address
  std::string ip_address_;

  /// Camera's UDP frame data port
  uint16_t frame_data_port_;

  /// Current publish tf state
  bool publish_tf_;

  /// current static tf values
  std::string parent_frame_;
  double x_;
  double y_;
  double z_;
  double roll_;
  double pitch_;
  double yaw_;

  /// time offset
  double time_offset_;

  /// Camera's frame configurations
  std::shared_ptr<hfl::Frame> frame_;

public:
  ///
  /// Gets the Model of the camera.
  ///
  /// @return string camera model
  ///
  std::string getModel() const;

  ///
  /// Gets the camera version.
  ///
  /// @return string camera model version
  ///
  std::string getVersion() const;

  ///
  /// Set the frame rate.
  ///
  /// @param[in] rate Camera frame rate
  ///
  /// @return bool true if given frame rate set
  ///
  virtual bool setFrameRate(double rate) = 0;

  ///
  /// Returns the current frame rate.
  ///
  /// @param[in] reg_format Indicates register format output
  /// @return current frame rate
  ///
  virtual double getFrameRate(bool reg_format = false) const = 0;

  ///
  /// Parse packet into depth and intensity image
  ///
  /// @param[in] start_byte starting byte, packet packet data to parse
  ///
  /// @return bool true if successfully parsed frame data
  ///
  virtual bool parseFrame(int start_byte, const std::vector<uint8_t>& packet) = 0;

  ///
  /// Process the frame data from udp packets
  ///
  /// @param[in] data frame data
  ///
  /// @return bool
  ///
  virtual bool processFrameData(const std::vector<uint8_t>& data) = 0;

  ///
  /// Reference to the frame_ member variable
  ///
  /// @return frame shared_ptr
  ///
  std::shared_ptr<Frame> frame();
};

}  // namespace hfl

#endif  // HFL_INTERFACE_H
