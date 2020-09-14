/// Copyright 2019 Continental AG
///
/// @file base_hfl110dcu.h
///
/// @brief This file defines the HFL110DCU camera base class.
///
#ifndef BASE_HFL110DCU_H
#define BASE_HFL110DCU_H
#include <hfl_interface.h>
#include <string>
#include <vector>

namespace hfl
{
/// Default frame rows
const uint16_t FRAME_ROWS{ 32 };
/// Default frame cols
const uint16_t FRAME_COLUMNS{ 128 };
/// Default frame cols
const uint16_t PIXEL_RETURNS{ 1 };
/// Default frame cols
const uint16_t PIXEL_SLICES{ 128 };
/// Default words per UDP packet
const uint32_t WORDS_PER_PACKET{ 0x168 };
///  Default bits used for intensity
const uint8_t INTENSITY_BITS{ 5 };
/// Default bits used for range
const uint8_t RANGE_BITS{ 8 };
/// Default bits used for range presicion
const uint8_t RANGE_PRECISION_BITS{ 6 };
/// Default bits used for intensity publishing
const uint8_t INTENSITY_PUBLISH_BITS{ 12 };
/// Default frame ID
const std::string FRAME_ID{ "hfl110dcu" };
/// Default camera intrinsics
const std::string CAMERA_INTRINSICS{ "min000000" };
/// Default expected memory address
const uint32_t EXPECTED_ADDRESS{ 0xffffffff };

///
/// @brief Base class for the HFL110DCU cameras
///
class BaseHFL110DCU : public HflInterface
{
public:
  ///
  /// Sets the specified frame rate.
  ///
  /// @param[in] rate Frame rate to be set
  ///
  /// @return bool true if given frame rate set
  ///
  bool setFrameRate(double rate) override
  {
    return false;
  }

  ///
  /// Returns the current frame rate.
  ///
  /// @return current frame rate
  ///
  double getFrameRate(bool reg_format = false) const
  {
    return 25.0;
  };

protected:
  /// Range Magic Number
  double range_magic_number_;

  /// Current mode parameters
  Attribs_map mode_parameters;

  /// UDP sender function
  std::function<void(const std::vector<uint8_t>&)> udp_send_function_;

  /// HFL110DCU camera memory_types
  enum HFL110DCU_memory_types
  {
    mem_ri = 0,
    types_size
  };

  ///
  /// Gets the available memory modes, its params and register
  /// offset addresses.
  ///
  /// @param model The model of the current lidar
  /// @param version The HFL110DCU SW version
  ///
  /// @return Available memory modes
  ///
  bool getConfiguration(std::string model, std::string version);
};
}  // namespace hfl

#endif  // BASE_HFL110DCU_H
