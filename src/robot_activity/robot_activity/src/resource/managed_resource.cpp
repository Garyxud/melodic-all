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
#include <robot_activity/resource/managed_resource.h>

namespace robot_activity
{
namespace resource
{

template<class Specialization, class Resource>
Managed<Specialization, Resource>::~Managed()
{
  ROS_DEBUG("Managed::dtor");
}

template<class Specialization, class Resource>
void Managed<Specialization, Resource>::acquire(const ros::NodeHandlePtr& node_handle)
{
  ROS_DEBUG("Managed::acquire executed!");
  if (acquired_)
  {
    ROS_DEBUG("Already acquired!");
    return;
  }

  ROS_DEBUG("Subscribing...");
  resource_ = lazy_acquirer_(node_handle);
  acquired_ = true;
}

template<class Specialization, class Resource>
void Managed<Specialization, Resource>::release()
{
  ROS_DEBUG("Managed::release executed!");
  if (acquired_)
  {
    ROS_DEBUG("Releasing...");
    resource_.shutdown();
    acquired_ = false;
  }
  else
  {
    ROS_DEBUG("Cannot release ");
  }
}

template<class Specialization, class Resource>
void Managed<Specialization, Resource>::pause()
{
  ROS_DEBUG("Managed::pause executed!");
  paused_ = true;
}

template<class Specialization, class Resource>
void Managed<Specialization, Resource>::resume()
{
  ROS_DEBUG("Managed::resume executed!");
  paused_ = false;
}

template class Managed<ManagedSubscriber, ros::Subscriber>;
template class Managed<ManagedServiceServer, ros::ServiceServer>;

}  // namespace resource
}  // namespace robot_activity
