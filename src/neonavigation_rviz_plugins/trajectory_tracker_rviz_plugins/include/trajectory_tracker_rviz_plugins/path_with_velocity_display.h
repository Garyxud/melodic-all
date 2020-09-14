/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TRAJECTORY_TRACKER_RVIZ_PLUGINS_PATH_WITH_VELOCITY_DISPLAY_H
#define TRAJECTORY_TRACKER_RVIZ_PLUGINS_PATH_WITH_VELOCITY_DISPLAY_H

#include <vector>

#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/PoseStampedWithVelocity.h>

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;
}  // namespace rviz

namespace trajectory_tracker_rviz_plugins
{
/**
 * \class PathWithVelocityDisplay
 * \brief Displays a trajectory_tracker_msgs::PathWithVelocity message
 */
class PathWithVelocityDisplay : public rviz::MessageFilterDisplay<trajectory_tracker_msgs::PathWithVelocity>
{
  Q_OBJECT
public:
  PathWithVelocityDisplay();
  virtual ~PathWithVelocityDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage(const trajectory_tracker_msgs::PathWithVelocity::ConstPtr& msg);

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowColor();
  void updatePoseArrowGeometry();

private:
  void destroyObjects();
  void allocateArrowVector(std::vector<rviz::Arrow*>& arrow_vect, size_t num);
  void allocateAxesVector(std::vector<rviz::Axes*>& axes_vect, size_t num);
  void destroyPoseAxesChain();
  void destroyPoseArrowChain();

  typedef std::vector<rviz::Axes*> AxesPtrArray;
  typedef std::vector<rviz::Arrow*> ArrowPtrArray;

  std::vector<Ogre::ManualObject*> manual_objects_;
  std::vector<rviz::BillboardLine*> billboard_lines_;
  std::vector<AxesPtrArray> axes_chain_;
  std::vector<ArrowPtrArray> arrow_chain_;

  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* line_width_property_;
  rviz::IntProperty* buffer_length_property_;
  rviz::VectorProperty* offset_property_;

  enum LineStyle
  {
    LINES,
    BILLBOARDS
  };

  // pose marker property
  rviz::EnumProperty* pose_style_property_;
  rviz::FloatProperty* pose_axes_length_property_;
  rviz::FloatProperty* pose_axes_radius_property_;
  rviz::ColorProperty* pose_arrow_color_property_;
  rviz::FloatProperty* pose_arrow_shaft_length_property_;
  rviz::FloatProperty* pose_arrow_head_length_property_;
  rviz::FloatProperty* pose_arrow_shaft_diameter_property_;
  rviz::FloatProperty* pose_arrow_head_diameter_property_;

  enum PoseStyle
  {
    NONE,
    AXES,
    ARROWS,
  };
};

}  // namespace trajectory_tracker_rviz_plugins

#endif  // TRAJECTORY_TRACKER_RVIZ_PLUGINS_PATH_WITH_VELOCITY_DISPLAY_H
