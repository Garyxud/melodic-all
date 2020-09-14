// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <pluginlib/class_list_macros.h>
#include "imagezero_image_transport/imagezero_publisher.h"
#include "imagezero_image_transport/imagezero_subscriber.h"

/*! \mainpage imagezero_image_transport
 *
 * This packages provides publisher and subscriber plugins for the
 * <a href="http://wiki.ros.org/image_transport">ROS image_transport</a> package.
 *
 * If you are looking for a way to convert between sensor_msgs::Image and
 * sensor_msgs::CompressedImage messages using the ImageZero compression algorithm,
 * you should look at the imagezero_ros package.
 *
 * If you are using image_transport, you do not need to directly use either of
 * these classes.  As long as this package is in your ROS package path, any
 * image_transport::Publishers will automatically publish an "imagezero" topic,
 * and any image_transport::Subscribers can be made to subscribe to it by
 * setting their ~image_transport parameter to "imagezero".
 */

PLUGINLIB_EXPORT_CLASS( imagezero_image_transport::ImageZeroPublisher, image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS( imagezero_image_transport::ImageZeroSubscriber, image_transport::SubscriberPlugin)
