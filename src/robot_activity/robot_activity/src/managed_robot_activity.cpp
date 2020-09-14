/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, University of Luxembourg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Luxembourg nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Maciej Zurad
 *********************************************************************/
#include <robot_activity/managed_robot_activity.h>

namespace robot_activity
{

ManagedRobotActivity::~ManagedRobotActivity()
{
  ROS_DEBUG_STREAM("ManagedRobotActivity dtor [" << getNamespace() << "]");
}

void ManagedRobotActivity::onCreate()
{
  ROS_DEBUG("onCreate");
  onManagedCreate();
}

void ManagedRobotActivity::onTerminate()
{
  ROS_DEBUG("onTerminate");
  onManagedTerminate();
}

void ManagedRobotActivity::onConfigure()
{
  ROS_DEBUG("onConfigure");
  onManagedConfigure();
}

void ManagedRobotActivity::onUnconfigure()
{
  ROS_DEBUG("onUnconfigure");
  onManagedUnconfigure();
}

void ManagedRobotActivity::onStart()
{
  ROS_DEBUG("onStart");
  service_manager.acquireAll(node_handle_private_);
  subscriber_manager.acquireAll(node_handle_private_);
  onManagedStart();
}

void ManagedRobotActivity::onStop()
{
  ROS_DEBUG("onStop");
  service_manager.releaseAll();
  subscriber_manager.releaseAll();
  onManagedStop();
}

void ManagedRobotActivity::onPause()
{
  ROS_DEBUG("onPause");
  service_manager.pauseAll();
  subscriber_manager.pauseAll();
  onManagedPause();
}

void ManagedRobotActivity::onResume()
{
  ROS_DEBUG("onResume");
  service_manager.resumeAll();
  subscriber_manager.resumeAll();
  onManagedResume();
}

}  // namespace robot_activity
