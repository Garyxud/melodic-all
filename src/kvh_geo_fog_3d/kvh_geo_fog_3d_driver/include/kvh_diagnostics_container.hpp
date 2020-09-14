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
 * @file kvh_diagnostics_container.hpp
 * @brief Contains diagnostic information published out the ROS
 * diagnostics topic.
 */

#pragma once

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace mitre
{
  namespace KVH
  {
    class DiagnosticsContainer
    {
    public:
      DiagnosticsContainer();

      ///////////////////////////////////////////////////////////////////////
      //All of these functions interpret and store data from messages.
      //They are NOT the functions called by the DiagnosticUpdater callbacks.
      ///////////////////////////////////////////////////////////////////////
      void SetSystemStatus(uint16_t _status) {systemStatus_ = _status; receivedSystemStatus_ = true;}
      void SetFilterStatus(uint16_t _status) {filterStatus_ = _status; receivedFilterStatus_ = true;}

      ///////////////////////////////////////////////////////////////////////
      // Callback functions for different diagnostics
      // Each of these may aggregate data from multiple systems
      ///////////////////////////////////////////////////////////////////////
      void UpdateSystemStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void UpdateFilterStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

    private:
      uint16_t systemStatus_{0xFFFF}; // Assume all errors until stated otherwise
      bool receivedSystemStatus_{false};
      uint16_t filterStatus_{0x0000}; // Assume no initialization until stated otherwise
      bool receivedFilterStatus_{false};
    }; //end: class DiagnosticsContainer
  } //end: namespace KVH
} //end: namespace mitre
