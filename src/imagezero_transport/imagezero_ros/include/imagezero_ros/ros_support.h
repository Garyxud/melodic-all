// *****************************************************************************
//
// Copyright (c) 2016-2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#ifndef IMAGEZERO_ROS_ROS_SUPPORT_H

#include <imagezero/portableimage.h>
#include <imagezero/libiz.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

namespace IZ
{
  static bool encode_tables_initialized = false;
  static bool decode_tables_initialized = false;

  /**
   * Uses the ImageZero algorithm to compress a raw Image into a CompressedImage.
   * The raw image must have 3 channels and either 8 or 16 bits per channel.
   * @param[in] image An uncompressed 24-bit color image.
   * @return A compressed image based on that image.
   */
  sensor_msgs::CompressedImage compressImage(const sensor_msgs::Image& image);

  /**
   * Decompresses an image that had previous been compressed using ImageZero
   * and returns a raw image.
   * @param[in] compressed An image that was compressed with ImageZero.
   * @return A raw image.
   */
  sensor_msgs::Image decompressImage(const sensor_msgs::CompressedImageConstPtr& compressed);
}

#define IMAGEZERO_ROS_ROS_SUPPORT_H

#endif //IMAGEZERO_ROS_ROS_SUPPORT_H
