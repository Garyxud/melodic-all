/// Copyright 2019 Continental AG
///
/// @file hfl_pixel.cpp
///
/// @brief This file implements the Frame component classes.
///

#include <hfl_pixel.h>

namespace hfl
{
Pixel::Pixel(size_t returns_size, size_t slices_size) : slices_(slices_size)
{
  PixelReturn pixelReturn;
  returns_.resize(returns_size, pixelReturn);
}

PixelReturn& Pixel::atReturn(uint16_t x)
{
  return returns_[x];
}

uint16_t& Pixel::atSlice(uint16_t x)
{
  return slices_[x];
}

}  // namespace hfl
