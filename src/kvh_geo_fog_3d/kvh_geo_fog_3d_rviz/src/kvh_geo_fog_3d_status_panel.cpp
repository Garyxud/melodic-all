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
 * @file kvh_geo_fog_3d_status_panel.cpp
 * @brief KVH Geo Fog 3D RVIZ plugin status panel.
 * @author Trevor Bostic
 *
 * This file implements all functions defined in kvh_geo_fog_3d_status_panel.hpp.
 * This RVIZ plugin is used to display sensor state in RVIZ.
 */

#include "kvh_geo_fog_3d_status_panel.hpp"

#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QFont>

#include "diagnostic_msgs/DiagnosticArray.h"
#include <functional>


namespace kvh
{
  /**
   * @fn StatusPanel::StatusPanel
   * @brief Constructor for our RVIZ panel
   * @param parent [in] Our QWidget parent to which we belong.
   *
   * Sets up our widget with layouts and titles. Also subscribes
   * to our ROS callback for diagnostics messages.
   */
  StatusPanel::StatusPanel(QWidget* parent)
    : rviz::Panel(parent)
  {
    QFont h1_font("Times", 18);
    QFont h2_font("Times", 14);

    //////////////////////////////////////////
    // System Status
    //////////////////////////////////////////
    QHBoxLayout* system_status_header = new QHBoxLayout;
    QLabel* sys_stat_title = new QLabel("System Status");
    sys_stat_title->setAlignment(Qt::AlignCenter);
    sys_stat_title->setFont(h1_font);
    system_status_header->addWidget(sys_stat_title);

    QHBoxLayout* system_status = new QHBoxLayout;
    QVBoxLayout* failures = new QVBoxLayout;
    QLabel* failure_label = new QLabel("Failures");
    failure_label->setAlignment(Qt::AlignCenter);
    failure_label->setFont(h2_font);
    failures->addWidget(failure_label);
    failures->addLayout(StatusIndicatorFactory(false, "System", "system"));
    failures->addLayout(StatusIndicatorFactory(false, "Accelerometers", "accelerometers_failure"));
    failures->addLayout(StatusIndicatorFactory(false, "Gyroscopes", "gyroscopes_failure"));
    failures->addLayout(StatusIndicatorFactory(false, "Magnetometers", "magnetometers_failure"));
    failures->addLayout(StatusIndicatorFactory(false, "Pressure", "pressure_failure"));
    failures->addLayout(StatusIndicatorFactory(false, "GNSS", "gnss"));
    system_status->addLayout(failures);

    QVBoxLayout* overrange = new QVBoxLayout;
    QLabel* overrange_label = new QLabel("Overrange");
    overrange_label->setAlignment(Qt::AlignCenter);
    overrange_label->setFont(h2_font);
    overrange->addWidget(overrange_label);
    overrange->addLayout(StatusIndicatorFactory(false, "Accelerometers", "accelerometers_overrange"));    
    overrange->addLayout(StatusIndicatorFactory(false, "Gyroscopes", "gyroscopes_overrange"));
    overrange->addLayout(StatusIndicatorFactory(false, "Magnetometers", "magnetometers_overrange"));
    overrange->addLayout(StatusIndicatorFactory(false, "Pressure", "pressure_overrange"));
    overrange->addWidget(new QLabel());
    overrange->addWidget(new QLabel());
    system_status->addLayout(overrange);

    QHBoxLayout* system_status_2 = new QHBoxLayout;
    QVBoxLayout* alarms = new QVBoxLayout;
    QLabel* alarms_label = new QLabel("Alarms");
    alarms_label->setAlignment(Qt::AlignCenter);
    alarms_label->setFont(h2_font);
    alarms->addWidget(alarms_label);
    alarms->addLayout(StatusIndicatorFactory(false, "Minimum Temperature", "minimum_temp"));
    alarms->addLayout(StatusIndicatorFactory(false, "Maximum Temperature", "maximum_temp"));
    alarms->addLayout(StatusIndicatorFactory(false, "Low Voltage", "low_volt"));
    alarms->addLayout(StatusIndicatorFactory(false, "High Voltage", "high_volt"));
    alarms->addLayout(StatusIndicatorFactory(false, "GNSS Antenna", "gps_antenna"));
    alarms->addLayout(StatusIndicatorFactory(false, "Serial Port Overflow", "serial_port_over"));
    system_status_2->addLayout(alarms);

    //////////////////////////////////////////////
    // Filter Status
    //////////////////////////////////////////////
    QHBoxLayout* filter_status_header = new QHBoxLayout;
    QLabel* filter_stat_title = new QLabel("Filter Status");
    filter_stat_title->setAlignment(Qt::AlignCenter);
    filter_stat_title->setFont(h1_font);
    filter_status_header->addWidget(filter_stat_title);

    QHBoxLayout* filter_status = new QHBoxLayout;
    QVBoxLayout* initialization = new QVBoxLayout;
    QLabel* initialization_label = new QLabel("Initialization");
    initialization_label->setAlignment(Qt::AlignCenter);
    initialization_label->setFont(h2_font);
    initialization->addWidget(initialization_label);
    initialization->addLayout(StatusIndicatorFactory(false, "Orientation", "orientation_init"));
    initialization->addLayout(StatusIndicatorFactory(false, "Navigation", "nav_init"));
    initialization->addLayout(StatusIndicatorFactory(false, "Heading", "heading_init"));
    initialization->addLayout(StatusIndicatorFactory(false, "Time", "time_init"));
    initialization->addWidget(new QLabel(""));
    initialization->addWidget(new QLabel(""));
    initialization->addWidget(new QLabel(""));
    filter_status->addLayout(initialization);

    QVBoxLayout* gnss_fix = new QVBoxLayout;
    QLabel* gnss_label = new QLabel("GNSS");
    gnss_label->setAlignment(Qt::AlignCenter);
    gnss_label->setFont(h2_font);
    gnss_fix->addWidget(gnss_label);
    gnss_fix->addLayout(StatusIndicatorFactory(false, "2D", "gnss_2D"));
    gnss_fix->addLayout(StatusIndicatorFactory(false, "3D", "gnss_3D"));
    gnss_fix->addLayout(StatusIndicatorFactory(false, "SBAS", "gnss_sbas"));
    gnss_fix->addLayout(StatusIndicatorFactory(false, "Differential", "gnss_diff"));
    gnss_fix->addLayout(StatusIndicatorFactory(false, "Omnistar", "gnss_omni"));
    gnss_fix->addLayout(StatusIndicatorFactory(false, "RTK Float", "gnss_rtk_float"));
    gnss_fix->addLayout(StatusIndicatorFactory(false, "RTK", "gnss_rtk"));
    filter_status->addLayout(gnss_fix);

    QHBoxLayout* filter_status_2 = new QHBoxLayout;
    QVBoxLayout* filter_sources = new QVBoxLayout;
    QLabel* filter_sources_label = new QLabel("Filter Sources");
    filter_sources_label->setAlignment(Qt::AlignCenter);
    filter_sources_label->setFont(h2_font);
    filter_sources->addWidget(filter_sources_label);
    filter_sources->addLayout(StatusIndicatorFactory(false, "Internal GNSS", "internal_gnss"));
    filter_sources->addLayout(StatusIndicatorFactory(false, "Dual Antenna Heading", "dual_antenna_head"));
    filter_sources->addLayout(StatusIndicatorFactory(false, "Velocity Heading", "vel_heading"));
    filter_sources->addLayout(StatusIndicatorFactory(false, "Atmospheric Altitude", "atm_alt"));
    filter_sources->addLayout(StatusIndicatorFactory(false, "External Position", "ext_pos"));
    filter_sources->addLayout(StatusIndicatorFactory(false, "External Velocity", "ext_vel"));
    filter_sources->addLayout(StatusIndicatorFactory(false, "External Heading", "ext_heading"));
    filter_status_2->addLayout(filter_sources);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(system_status_header);
    layout->addLayout(system_status);
    layout->addLayout(system_status_2);
    layout->addLayout(filter_status_header);
    layout->addLayout(filter_status);
    layout->addLayout(filter_status_2);

    diag_sub_ = nh_.subscribe("diagnostics", 1, &StatusPanel::DiagnosticsCallback, this);
        
    setLayout(layout);
  }

  /**
   * @fn StatusPanel::StatusIndicatorFactory
   * @brief Create mappings from our diagnostics strings to painters.
   * @param _enabled [in] If we're showing this status
   * @param _label [in] The label of this status section
   * @param _map_key [in] The string to use as the key into the map of painters
   * @return A Qt layout containing our new status objects and painters.
   */
  QHBoxLayout* StatusPanel::StatusIndicatorFactory(bool _enabled, std::string _label, std::string _map_key)
  {
    QHBoxLayout* layout = new QHBoxLayout;
    StatusPainter* painter = new StatusPainter;
    painter->setEnabled(_enabled);

    painter_map_[_map_key] = painter;

    layout->addWidget(new QLabel(_label.c_str()));
    layout->addWidget(painter);

    return layout;
  }

  /**
   * @fn StatusPanel::DiagnosticsCallback
   * @brief Function to receive diagnostics information and calculate statuses
   * @param _diagArray The diagnostics message to operate on
   *
   * Does the work of parsing out diagnostics information and filling in our
   * map.
   */
  void StatusPanel::DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& _diagArray)
  {
    ROS_DEBUG("Recieved diagnostic message.");
    int kvh_filters_index{0};
    bool kvh_filters_found{false};
        
    int kvh_system_index{0};
    bool kvh_system_found{false};

    int loopCount{0};
    for(diagnostic_msgs::DiagnosticStatus status : _diagArray->status)
    {
      if(status.name == "kvh_geo_fog_3d_driver_node: KVH Filters" && status.hardware_id == "KVH GEO FOG 3D")
      {
        kvh_filters_found = true;
        kvh_filters_index = loopCount;
      }
      else if(status.name == "kvh_geo_fog_3d_driver_node: KVH System" && status.hardware_id == "KVH GEO FOG 3D")
      {
        kvh_system_found = true;
        kvh_system_index = loopCount;
      }
      loopCount++;
    }

    if (kvh_filters_found)
    {
      diagnostic_msgs::DiagnosticStatus kvh_status = _diagArray->status[kvh_filters_index];

      for (diagnostic_msgs::KeyValue pair : kvh_status.values)
      {
        ROS_DEBUG("%s, %s", pair.key.c_str(), pair.value.c_str());
        if(pair.key == "Orient INIT")
        {
          painter_map_["orientation_init"]->setEnabled(pair.value == "True");
        } 
        else if(pair.key == "Nav INIT")
        {
          painter_map_["nav_init"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Heading INIT")
        {
          painter_map_["heading_init"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Time INIT")
        {
          painter_map_["time_init"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Internal GNSS Enabled")
        {
          painter_map_["internal_gnss"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Dual ANT Heading Active")
        {
          painter_map_["dual_antenna_head"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Vel Heading Enabled")
        {
          painter_map_["vel_heading"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Atm Alt Enabled")
        {
          painter_map_["atm_alt"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "EXT Pos Active")
        {
          painter_map_["ext_pos"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "EXT Vel Active")
        {
          painter_map_["ext_vel"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "EXT Heading Active")
        {
          painter_map_["ext_heading"]->setEnabled(pair.value == "True");
        }
        else if(pair.key == "Fix Status")
        {
          painter_map_["gnss_2D"]->setEnabled(pair.value == "2D Fix");
          painter_map_["gnss_3D"]->setEnabled(pair.value == "3D Fix");
          painter_map_["gnss_sbas"]->setEnabled(pair.value == "SBAS");
          painter_map_["gnss_diff"]->setEnabled(pair.value == "Diff");
          painter_map_["gnss_omni"]->setEnabled(pair.value == "Omni/Star");
          painter_map_["gnss_rtk_float"]->setEnabled(pair.value == "RTK Float");
          painter_map_["gnss_rtk"]->setEnabled(pair.value == "RTK Fixed");
        }
        else if(pair.key == "Event1")
        {
          // Ignore for now
        }
        else if(pair.key == "Event2")
        {
          // Ignore for now
        }
        else
        {
          ROS_ERROR("CODING ERROR! You shouldn't be able to get here.");
        }
      }
    }

    if (kvh_system_found)
    {
      diagnostic_msgs::DiagnosticStatus kvh_system_status = _diagArray->status[kvh_system_index];

      for (diagnostic_msgs::KeyValue pair : kvh_system_status.values)
      {
        if (pair.key == "System")
        {
          painter_map_["system"]->setEnabled(pair.value == "False");                    
        }
        else if (pair.key == "Accel")
        {
          painter_map_["accelerometers_failure"]->setEnabled(pair.value == "False");  
        }
        else if (pair.key == "Gyro")
        {
          painter_map_["gyroscopes_failure"]->setEnabled(pair.value == "False");  
        }
        else if (pair.key == "Mag")
        {
          painter_map_["magnetometers_failure"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Pressure")
        {
          painter_map_["pressure_failure"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "GNSS")
        {
          painter_map_["gnss"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Acc OOR")
        {
          painter_map_["accelerometers_overrange"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Gyro OOR")
        {
          painter_map_["gyroscopes_overrange"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Mag OOR")
        {
          painter_map_["magnetometers_overrange"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Pressure OOR")
        {
          painter_map_["pressure_overrange"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Min Temp")
        {
          painter_map_["minimum_temp"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Max Temp")
        {
          painter_map_["maximum_temp"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Low V")
        {
          painter_map_["low_volt"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "High V")
        {
          painter_map_["high_volt"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "GNSS ANT DC")
        {
          painter_map_["gps_antenna"]->setEnabled(pair.value == "False");
        }
        else if (pair.key == "Data Overflow")
        {
          painter_map_["serial_port_over"]->setEnabled(pair.value == "False");
        }
      }
    }

  }


}

// Exporting the plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kvh::StatusPanel, rviz::Panel);
