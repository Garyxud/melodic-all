/// Copyright 2019 Continental AG
///
/// @file hfl_configs.h
///
/// @brief This file defines HFL cameras data and custom types.
///
#ifndef HFL_CONFIGS_H
#define HFL_CONFIGS_H

#include <iostream>
#include <string>
#include <functional>
#include <vector>
#include <map>
#include <memory>
#include <utility>

namespace hfl
{
///  Mode parameters map
using Attribs_map = std::map<std::string, float>;

/// Camera modes map
using Configs_map = std::map<std::string, Attribs_map>;

/// HFL cameras map
using Setups_map = std::map<std::string, Configs_map>;

/// Register bit's values vector
using Regs_bits_vec = std::vector<std::pair<std::string, int>>;

/// Cameras registers map
using Registers_map = std::map<std::string, Regs_bits_vec>;

/// HFL cameras register addresses
const Configs_map REGS_OFFSET_ADDRS
{
  { "hfl110dcu", {} }
};

/// HFL cameras memory maps and parameters
const Setups_map CAMERA_MODELS =
{
  { "hfl110dcuv1",
    {
      { "RI",
        {
          { "start_address", 0x00000000 }
        }
      }
    }
  }
  // Insert new camera models here
};

/// HFL cameras mode registers
const Registers_map MODE_REGISTERS =
{
  { "hfl110dcuv1",
    {
        // BLANK
    }
  }
};

}  // namespace hfl

#endif  //  HFL_CONFIGS_H
