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
 * @file kvh_status_painter.hpp
 * @brief KVH Geo Fog 3D RVIZ plugin painter object.
 * @author Trevor Bostic
 *
 * This file defines our RVIZ panel painter object for
 * the KVH status display.
 */

#pragma once

#include <QWidget>
#include <QPainter>

namespace kvh
{
  /**
   * @class StatusPainter
   * @ingroup kvh
   * @brief StatusPainter which lets us draw red/green
   * circles for device status.
   */
  class StatusPainter : public QWidget
  {
    Q_OBJECT
        
  public:
    StatusPainter(QWidget* parent = 0);
    virtual void paintEvent(QPaintEvent* event);

  };

}
