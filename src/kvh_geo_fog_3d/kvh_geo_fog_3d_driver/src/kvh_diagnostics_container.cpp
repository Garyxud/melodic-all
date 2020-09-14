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
 * @file kvh_diagnostics_container.cpp
 * @brief Wrapper around ROS diagnostics information for the KVH
 * @author Trevor Bostic
 */

#include "kvh_diagnostics_container.hpp"

mitre::KVH::DiagnosticsContainer::DiagnosticsContainer()
{

} //end: DiagnosticsContainer()

void mitre::KVH::DiagnosticsContainer::UpdateSystemStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if( receivedSystemStatus_ )
  {
    if( systemStatus_ == 0 )
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System State OK");
      stat.add("System", false);
      stat.add("Accel", false);
      stat.add("Gyro", false);
      stat.add("Mag", false);
      stat.add("Pressure", false);
      stat.add("GNSS", false);
      stat.add("Acc OOR", false);
      stat.add("Gyro OOR", false);
      stat.add("Mag OOR", false);
      stat.add("Pressure OOR", false);
      stat.add("Min Temp", false);
      stat.add("Max Temp", false);
      stat.add("Low V", false);
      stat.add("High V", false);
      stat.add("GNSS ANT DC", false);
      stat.add("Data Overflow", false);
    } //end: if( systemStatus_ == 0 )
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "System FAULT");
      if( systemStatus_ & 0x1 )
      {
        stat.add("System", true);
      }
      else
      {
        stat.add("System", false);
      }
      if( systemStatus_ & 0x2 )
      {
        stat.add("Accel", true);
      }
      else
      {
        stat.add("Accel", false);
      }
      if( systemStatus_ & 0x4 )
      {
        stat.add("Gyro", true);
      }
      else
      {
        stat.add("Gyro", false);
      }
      if( systemStatus_ & 0x8 )
      {
        stat.add("Mag", true);
      }
      else
      {
        stat.add("Mag", false);
      }
      if( systemStatus_ & 0x10 )
      {
        stat.add("Pressure", true);
      }
      else
      {
        stat.add("Pressure", false);
      }
      if( systemStatus_ & 0x20 )
      {
        stat.add("GNSS", true);
      }
      else
      {
        stat.add("GNSS", false);
      }
      if( systemStatus_ & 0x40 )
      {
        stat.add("Acc OOR", true);
      }
      else
      {
        stat.add("Acc OOR", false);
      }
      if( systemStatus_ & 0x80 )
      {
        stat.add("Gyro OOR", true);
      }
      else
      {
        stat.add("Gyro OOR", false);
      }
      if( systemStatus_ & 0x100 )
      {
        stat.add("Mag OOR", true);
      }
      else
      {
        stat.add("Mag OOR", false);
      }
      if( systemStatus_ & 0x200 )
      {
        stat.add("Pressure OOR", true);
      }
      else
      {
        stat.add("Pressure OOR", false);
      }
      if( systemStatus_ & 0x400 )
      {
        stat.add("Min Temp", true);
      }
      else
      {
        stat.add("Min Temp", false);
      }
      if( systemStatus_ & 0x800 )
      {
        stat.add("Max Temp", true);
      }
      else
      {
        stat.add("Max Temp", false);
      }
      if( systemStatus_ & 0x1000 )
      {
        stat.add("Low V", true);
      }
      else
      {
        stat.add("Low V", false);
      }
      if( systemStatus_ & 0x2000 )
      {
        stat.add("High V", true);
      }
      else
      {
        stat.add("High V", false);
      }
      if( systemStatus_ & 0x4000 )
      {
        stat.add("GNSS ANT DC", true);
      }
      else
      {
        stat.add("GNSS ANT DC", false);
      }
      if( systemStatus_ & 0x8000 )
      {
        stat.add("Data Overflow", true);
      }
      else
      {
        stat.add("Data Overflow", false);
      }
    } //end: else
  } //end: if( receivedSystemStatus )
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "Waiting on initial SystemState packet");
  } //end: else
} //end: UpdateSystemStatus()

void mitre::KVH::DiagnosticsContainer::UpdateFilterStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  if( receivedFilterStatus_ )
  {
    ///////////////////////////////////////////////////////
    //For each filter, check to see if they're initialized.
    //If they are not, set the level to WARN
    bool currentValue = filterStatus_ & 0x1;
    if( currentValue == false )
    {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    stat.add("Orient INIT", currentValue);

    currentValue = filterStatus_ & 0x2;
    if( currentValue == false )
    {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    stat.add("Nav INIT", currentValue);

    currentValue = filterStatus_ & 0x4;
    if( currentValue == false )
    {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    stat.add("Heading INIT", currentValue);

    currentValue = filterStatus_ & 0x8;
    if( currentValue == false )
    {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    stat.add("Time INIT", currentValue);
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // Check GNSS Fix status
    // No fix is warn, everything else is OK
    // These 3 bits are 0x10, 0x20, and 0x40 (so 0x70 for all 3) then shift right to remove zeros
    std::string currentStatusStr("");
    switch(static_cast<int>((filterStatus_ & 0x70) >> 4))
    {
      case 0:
      {
        level = diagnostic_msgs::DiagnosticStatus::WARN;
        currentStatusStr = std::string("No Fix");
        break;
      }
      case 1:
      {
        currentStatusStr = std::string("2D Fix");
        break;
      }
      case 2:
      {
        currentStatusStr = std::string("3D Fix");
        break;
      }
      case 3:
      {
        currentStatusStr = std::string("SBAS");
        break;
      }
      case 4:
      {
        currentStatusStr = std::string("Diff");
        break;
      }
      case 5:
      {
        currentStatusStr = std::string("Omni/Star");
        break;
      }
      case 6:
      {
        currentStatusStr = std::string("RTK Float");
        break;
      }
      case 7:
      {
        currentStatusStr = std::string("RTK Fixed");
        break;
      }
      default:
      {
        currentStatusStr = std::string("CODING ERROR: YOU CAN'T GET HERE!!!");
        break;
      }
    } //end: switch(static_cast<int>(filterStatus_ & 0x70))
    stat.add("Fix Status", currentStatusStr);
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // These all don't effect status, but we will report them
    bool event1 = filterStatus_ & 0x80;
    bool event2 = filterStatus_ & 0x100;
    bool internalGNSSEnabled = filterStatus_ & 0x200;
    bool dualAntennaHeadingActive = filterStatus_ & 0x400;
    bool velocityHeadingEnabled = filterStatus_ & 0x800;
    bool atmosphericAltitudeEnabled = filterStatus_ & 0x1000;
    bool extPositionActive = filterStatus_ & 0x2000;
    bool extVelocityActive = filterStatus_ & 0x4000;
    bool extHeadingActive = filterStatus_ & 0x8000;
    stat.add("Event1", event1);
    stat.add("Event2", event2);
    stat.add("Internal GNSS Enabled", internalGNSSEnabled);
    stat.add("Dual ANT Heading Active", dualAntennaHeadingActive);
    stat.add("Vel Heading Enabled", velocityHeadingEnabled);
    stat.add("Atm Alt Enabled", atmosphericAltitudeEnabled);
    stat.add("EXT Pos Active", extPositionActive);
    stat.add("EXT Vel Active", extVelocityActive);
    stat.add("EXT Heading Active", extHeadingActive);
    ///////////////////////////////////////////////////////

    if( level == diagnostic_msgs::DiagnosticStatus::OK )
    {
      stat.summary(level, "Filters OK");
    }
    else
    {
      stat.summary(level, "Filters have issues");
    }
  } //end: if( receivedFilterStatus_ )
  else
  {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    stat.summary(level, "Waiting on initial FilterStatus packet");
  } //end: else
} //end: UpdateFilterStatus()
