/// Copyright 2019 Continental AG
///
/// @file hfl_interface.cpp
///
/// @brief This file defines HFL cameras base class.
///

#include <hfl_interface.h>
#include <string>

// Implementation for the hfl_utils functions

namespace hfl
{

std::string HflInterface::getModel() const
{
  return model_;
}

std::string HflInterface::getVersion() const
{
  return version_;
}

std::shared_ptr<Frame> HflInterface::frame()
{
  return frame_;
}

}  // namespace hfl
