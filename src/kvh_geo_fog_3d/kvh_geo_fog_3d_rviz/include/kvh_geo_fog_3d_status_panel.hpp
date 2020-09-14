/*********************************************************************
 * Software License Agreement (Apache 2.0)
 * 
 *  Copyright (c) 2019, The MITRE Corporation.
 *  All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Sections of this project contains content developed by The MITRE Corporation.
 * If this code is used in a deployment or embedded within another project,
 * it is requested that you send an email to opensource@mitre.org in order to
 * let us know where this software is being used.
 *********************************************************************/

/**
 * @file kvh_geo_fog_3d_status_panel.hpp
 * @brief KVH Geo Fog 3D RVIZ plugin status panel.
 * @author Trevor Bostic
 *
 * This file defines our RVIZ panel for displaying KVH sensor status.
 */

#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QHBoxLayout>
#include <unordered_map>
#include "diagnostic_msgs/DiagnosticArray.h"

#include "kvh_status_painter.hpp"

namespace kvh 
{
  /**
   * @class StatusPanel
   * @ingroup kvh
   * @brief RVIZ panel for displaying KVH sensor status.
   */
  class StatusPanel : public rviz::Panel
  {
    Q_OBJECT

  public:
    StatusPanel(QWidget* parent = 0);
    QHBoxLayout* StatusIndicatorFactory(bool, std::string, std::string);
    void DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr&);
  protected:
    ros::NodeHandle nh_; ///< Our ROS nodehandle object
    std::unordered_map<std::string, StatusPainter*> painter_map_; ///< Holds our key/value set of diagnostics message strings to painter objects
    ros::Subscriber diag_sub_; ///< ROS subscriber for diagnostics information

  };

}
