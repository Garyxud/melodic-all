/// Copyright 2019 Continental AG
///
/// @file base_hfl110dcu.cpp
///
/// @brief This file defines HFL110DCU cameras base class.
///

#include <base_hfl110dcu.h>
#include <string>

namespace hfl
{
bool BaseHFL110DCU::getConfiguration(std::string model, std::string version)
{
  std::string full_model = model + version;
  // Return false if camera registers not found
  if (REGS_OFFSET_ADDRS.find(model) == REGS_OFFSET_ADDRS.end())
  {
    std::cout << "[ERROR]" << model << " not available" << std::endl;
    return false;
  }
  // Return false if model not found
  if (CAMERA_MODELS.find(full_model) == CAMERA_MODELS.end())
  {
    std::cout << "[ERROR]"
              << " configuration registers for " << version << " not available" << std::endl;
    return false;
  }
  // Return false if mode register not found
  if (MODE_REGISTERS.find(full_model) == MODE_REGISTERS.end())
  {
    std::cout << "[ERROR]" << full_model << " mode register not available" << std::endl;
    return false;
  }
  // Set current model and version
  model_ = model;
  version_ = version;
  // Set frame configurations
  frame_.reset(new Frame(FRAME_ROWS, FRAME_COLUMNS, PIXEL_RETURNS, PIXEL_SLICES));
  frame_->intensity_bits_ = INTENSITY_BITS;
  frame_->range_bits_ = RANGE_BITS;
  frame_->range_precision_bits_ = RANGE_PRECISION_BITS;
  frame_->intensity_publish_bits_ = INTENSITY_PUBLISH_BITS;
  frame_->id_ = FRAME_ID;
  // frame_ changed to frame_ as referenced in the hfl_interface.h class
  return true;
}

}  // namespace hfl
