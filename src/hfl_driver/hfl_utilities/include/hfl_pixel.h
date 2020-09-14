/// Copyright 2019 Continental AG
///
/// @file hfl_pixel.h
///
/// @brief This file defines the frame's classes components.
///

#ifndef HFL_PIXEL_H
#define HFL_PIXEL_H

#include <iostream>
#include <vector>

namespace hfl
{
/// Pixel's Slice data type
using Slice = std::vector<uint16_t>;

///
/// @brief Data structure for pixel returns.
///
struct PixelReturn
{
  /// Return's range value
  float range{ 0.0 };

  /// Return's intensity value
  uint16_t intensity{ 0 };
};

///
/// @brief Storages and handles the pixel data component.
///
class Pixel
{
public:
  ///
  /// Pixel initializer constructor
  ///
  /// @param returns_size Number of returns per pixel
  /// @param slices_size Number of slices per pixel
  ///
  Pixel(size_t returns_size, size_t slices_size);

  ///
  /// Return the Pixel's return data at location x
  ///
  /// @param x Desired pixel return
  /// @return PixelReturn data structure
  ///
  PixelReturn& atReturn(uint16_t x);

  ///
  /// Returns the Slice data value at location x
  ///
  /// @param x Desired Slice position
  /// @return Slice data value
  ///
  uint16_t& atSlice(uint16_t x);

private:
  /// Slices data
  Slice slices_;

  /// Pixel's returns data
  std::vector<PixelReturn> returns_;
};

}  // namespace hfl
#endif  // HFL_PIXEL_H
