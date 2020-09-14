/// Copyright 2019 Continental AG
///
/// @file hfl_frame.h
///
/// @brief This file defines the camera's Frame class.
///

#ifndef HFL_FRAME_H
#define HFL_FRAME_H

#include <string>
#include <vector>
#include <hfl_pixel.h>

namespace hfl
{
/// Row number data type
using Row = uint16_t;

/// Column number data type
using Col = uint16_t;

///
/// @brief Handles camera's frame data.
///
class Frame
{
public:
  /// Frame ID
  std::string id_;

  /// Frame intensity bits
  uint16_t intensity_bits_;

  /// Frame intensity bits
  uint16_t range_bits_;

  /// Frame intensity bits
  uint16_t range_precision_bits_;

  /// Frame intensity bits
  uint16_t intensity_publish_bits_;

  ///
  /// Frame initializator constructor
  ///
  /// @param height Frame number of rows
  /// @param width  Frame number of columns
  /// @param returns_size Number of returns per pixel
  /// @param slices_size Number of slices per pixel
  ///
  Frame(uint16_t height, uint16_t width, uint16_t returns_size, uint16_t slices_size);

  ///
  /// Returns Pixel data structure at locatio (y,x)
  ///
  /// @param x Column position
  /// @param y Row position
  ///
  /// @return Pixel data
  ///
  Pixel& atPixel(Col x, Row y);

  ///
  /// Returns frame number of rows
  ///
  /// @return size_t frame height
  ///
  size_t getHeight() const
  {
    return height_;
  }

  ///
  /// Returns frame number of columns
  ///
  /// @return size_t frame width
  ///
  size_t getWidth() const
  {
    return width_;
  }

  ///
  /// Returns frame size (height*width)
  ///
  /// @return size_t frame width
  ///
  size_t getSize() const
  {
    return height_ * width_;
  }

  ///
  /// Returns frame size (height*width)
  ///
  /// @return size_t frame width
  ///
  size_t getSlicesSize() const
  {
    return slices_size_;
  }

private:
  /// Number of rows
  size_t height_;

  /// Number of columns
  size_t width_;

  /// Number of returns per pixel
  size_t returns_size_;

  /// Number of slices per pixel
  size_t slices_size_;

  /// Pixel data array
  std::vector<std::vector<Pixel>> pixels;
};

}  // namespace hfl

#endif  // HFL_FRAME_H
