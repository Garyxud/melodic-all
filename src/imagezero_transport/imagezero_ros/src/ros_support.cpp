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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>
#include <imagezero_ros/ros_support.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

/*! \mainpage imagezero_ros
 *
 * This package contains convenience methods for converting back and forth
 * between sensor_msgs::Image messages and sensor_msgs::CompressedImage messages.
 *
 * The ImageZero algorithm currently only operates on 24-bit PPM data, so under
 * the hood this uses OpenCV to convert messages into an appropriate format
 * and then run it through the algorithms in the imagezero package.
 */

namespace enc = sensor_msgs::image_encodings;

namespace IZ
{
  sensor_msgs::CompressedImage compressImage(const sensor_msgs::Image& image)
  {
    if (!encode_tables_initialized) {
      encode_tables_initialized = true;
      IZ::initEncodeTable();
    }

    // Compressed image message
    sensor_msgs::CompressedImage compressed;
    compressed.header = image.header;
    compressed.format = image.encoding;

    // Compression settings
    std::vector<int> params;

    // Bit depth of image encoding
    int bitDepth = enc::bitDepth(image.encoding);

    params.push_back(CV_IMWRITE_PXM_BINARY);
    params.push_back(1);

    // Update ros message format header
    compressed.format += "; iz compressed ";

    // Check input format
    if ((bitDepth == 8) || (bitDepth == 16))
    {

      // Target image format
      stringstream targetFormat;
      if (enc::isColor(image.encoding))
      {
        // convert color images to RGB domain
        targetFormat << "bgr" << bitDepth;
        compressed.format += targetFormat.str();
      }

      // OpenCV-ros bridge
      try
      {
        boost::shared_ptr<sensor_msgs::CompressedImagePtr> tracked_object;
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image, tracked_object, targetFormat.str());

        IZ::PortableImage pi;

        // Compress image
        std::vector<unsigned char> ppm_buffer;
        ros::Time now = ros::Time::now();
        if (cv::imencode(".ppm", cv_ptr->image, ppm_buffer, params))
        {
          if (!pi.readHeader((const unsigned char*) &ppm_buffer[0]))
          {
            std::stringstream ss;
            for (int i = 0; i < 20; i++)
            {
              ss << ppm_buffer[i];
            }
            ROS_ERROR("Unable to read PPM produced by OpenCV.  First few bytes: %s", ss.str().c_str());
            return sensor_msgs::CompressedImage();
          }
          if (pi.components() != 3)
          {
            ROS_ERROR("Can only handle 24-bit PPM files.");
            return sensor_msgs::CompressedImage();
          }
          unsigned char *dest = static_cast<unsigned char*>(malloc(pi.height() * pi.width() * 4 + 33));
          //IZ::initEncodeTable();
          unsigned char *destEnd = IZ::encodeImage(pi, dest);
          size_t size = destEnd - dest;
          compressed.data.resize(size);
          memcpy(&compressed.data[0], dest, size);
          free(dest);
          ros::Time iz_time = ros::Time::now();
          ROS_DEBUG_THROTTLE(1, "Took %lums to compress %lu bytes to %lu bytes (%f ratio)",
                             (iz_time - now).toNSec()/1000000L,
                             image.data.size(),
                             compressed.data.size(),
                             (double)image.data.size() / (double)image.data.size());
        }
        else
        {
          ROS_ERROR("cv::imencode (ppm) failed on input image");
          return sensor_msgs::CompressedImage();
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return sensor_msgs::CompressedImage();
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return sensor_msgs::CompressedImage();
      }
      
      return compressed;
    }
    else
    {
      ROS_ERROR(
          "Compressed Image Transport - ImageZero compression requires 8/16-bit encoded color format (input format is: %s)",
          image.encoding.c_str());
    }

    return sensor_msgs::CompressedImage();
  }

  sensor_msgs::Image decompressImage(const sensor_msgs::CompressedImageConstPtr& compressed)
  {
    if (!decode_tables_initialized){
      decode_tables_initialized = true;
      IZ::initDecodeTable();
    }

    IZ::PortableImage pi;
    //IZ::initDecodeTable();
    IZ::decodeImageSize(pi, &compressed->data[0]);
    pi.setComponents(3);
    const unsigned int dataSize = pi.width() * pi.height() * pi.components();
    unsigned char* dest = (unsigned char*) malloc(dataSize + 33);
    pi.writeHeader(dest);
    IZ::decodeImage(pi, &compressed->data[0]);

    std::vector<unsigned char> ppm_data;
    size_t size = pi.data() - dest + dataSize;
    ppm_data.resize(size);
    memcpy(&ppm_data[0], dest, size);
    free(dest);

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Copy compressed header
    cv_ptr->header = compressed->header;

    // Decode color/mono image
    try
    {
      cv_ptr->image = cv::imdecode(cv::Mat(ppm_data), CV_LOAD_IMAGE_COLOR);

      // Assign image encoding string
      const size_t split_pos = compressed->format.find(';');
      if (split_pos == std::string::npos)
      {
        switch (cv_ptr->image.channels())
        {
          case 1:
            cv_ptr->encoding = enc::MONO8;
            break;
          case 3:
            cv_ptr->encoding = enc::BGR8;
            break;
          default:
            ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
            break;
        }
      }
      else
      {
        std::string image_encoding = compressed->format.substr(0, split_pos);

        cv_ptr->encoding = image_encoding;

        if (enc::isColor(image_encoding))
        {
          std::string compressed_encoding = compressed->format.substr(split_pos);
          bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

          // Revert color transformation
          if (compressed_bgr_image)
          {
            // if necessary convert colors from bgr to rgb
            if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
            {
              cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);
            }

            if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            {
              cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);
            }

            if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            {
              cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
            }
          }
          else
          {
            // if necessary convert colors from rgb to bgr
            if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
            {
              cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
            }

            if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            {
              cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);
            }

            if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            {
              cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
            }
          }
        }
      }
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("%s", e.what());
    }

    size_t rows = cv_ptr->image.rows;
    size_t cols = cv_ptr->image.cols;

    if ((rows > 0) && (cols > 0))
    {
      // Publish message to user callback
      sensor_msgs::Image msg;
      cv_ptr->toImageMsg(msg);
      return msg;
    }

    return sensor_msgs::Image();
  }
}
