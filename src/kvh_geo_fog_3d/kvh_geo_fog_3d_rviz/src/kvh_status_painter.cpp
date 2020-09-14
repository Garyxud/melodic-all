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
 * @file kvh_status_painter.cpp
 * @brief KVH Geo Fog 3D RVIZ plugin painter object.
 * @author Trevor Bostic
 *
 * This file implements our painter for the RVIz plugin
 * to display status.
 */

#include "kvh_status_painter.hpp"

namespace kvh
{
  /**
   * @fn StatusPainter::StatusPainter
   * @brief Initialize our Qt StatusPainter
   *
   * @param parent [in] The QWidget to which
   * this painter belongs.
   */
  StatusPainter::StatusPainter(QWidget* parent)
    : QWidget(parent)
  {
  }

  /**
   * @fn StatusPainter::paintEvent
   * @brief The custom paintEvent for our QWidget
   *
   * @param event [in] the paint event object we're executing
   *
   * This custom painter draws green circles if things are ok
   * and red circles if things are bad.
   */
  void StatusPainter::paintEvent(QPaintEvent* event)
  {
    QPainter painter(this);
    QColor enabled;

    if (isEnabled())
    {
      enabled = Qt::green;
    }
    else
    {
      enabled = Qt::red;
    }
        

    painter.setBrush(enabled);

    int w = width();
    int h = height();

    painter.drawEllipse(0, h/2-3, 10, 10);
  }
}
