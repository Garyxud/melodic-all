/// Copyright 2019 Continental AG
///
/// @file hfl_frame.cpp
///
/// @brief This file implements the Frame class.
///

#include <hfl_frame.h>
#include <vector>

namespace hfl
{
Frame::Frame(uint16_t height, uint16_t width, uint16_t returns_size, uint16_t slices_size)
  : height_(height), width_(width), returns_size_(returns_size), slices_size_(slices_size)
{
  Pixel pixel(returns_size_, slices_size_);
  pixels.resize(height_, std::vector<Pixel>(width_, pixel));
}

Pixel& Frame::atPixel(Col x, Row y)
{
  return pixels[y][x];
}

}  // namespace hfl
