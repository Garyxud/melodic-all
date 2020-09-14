/**
Software License Agreement (BSD)

\file      warthog_suspension_plugin.cpp
\authors   Peiyi Chen <pchen@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "ros/ros.h"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include <boost/bind.hpp>
#include <string>

namespace gazebo
{
  /*
   * Define joint names.
   */
  static const std::string L_SUS_JNT = "left_diff_unit_joint";
  static const std::string R_SUS_JNT = "right_diff_unit_joint";

  class WarthogSuspensionPlugin : public ModelPlugin
  {
  public:
    /**
     * Constructor function.
     */
    WarthogSuspensionPlugin()
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "warthog_suspension_plugin");

      // Initialize last_update_time_
      last_update_time_ = ros::Time::now();
    }

    /**
     * Function executed when the plugin is loaded.
     * @param _parent Pointer to the physics::Model object containing the links and joints.
     * @param _sdf Pointer to the sdf::Element object containing the plugin parameters.
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Save the pointer to the model
      this->model_ = _parent;

      // Load parameters
      sus_kf_ = _sdf->Get<double>("suspension_k");
      sus_bf_ = _sdf->Get<double>("suspension_b");

      // Retrieve the model joints
      suspension_jnts_["left"] = model_->GetJoint(L_SUS_JNT);
      suspension_jnts_["right"] = model_->GetJoint(R_SUS_JNT);

      // Listen for update event (every simulation iteration/step)
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&WarthogSuspensionPlugin::OnUpdate, this, _1) );
    }

    /**
     * Function that is executed every time the simulation updates (i.e. each simulation step).
     * @param _info Object containing information regarding the current simulation step.
     */
    void OnUpdate(const common::UpdateInfo& _info)
    {
      // Calculate time interval
      ros::Time curr_time = ros::Time::now();
      double elapsed_t = (curr_time - last_update_time_).toSec();
      last_update_time_ = curr_time;

      // Apply the suspension forces based on current vehicle state
      applySuspension(sus_kf_, sus_bf_);
    }

  private:
  /**
   * Function that calculates the force applied to each suspension joint based on a spring/damper model
   * @param kf the spring constant in N/m
   * @param bf the dampening coefficient in Ns/m
    */
  void applySuspension(double kf, double bf)
  {
    // Get the displacement and velocity of the left and right suspensions
    double l_d = suspension_jnts_["left"]->Position();
    double r_d = suspension_jnts_["right"]->Position();

    double l_v = suspension_jnts_["left"]->GetVelocity(0);
    double r_v = suspension_jnts_["right"]->GetVelocity(0);

    // Set spring and damper forces on each suspension
    suspension_jnts_["left"]->SetForce(0, -1*(kf*l_d)-(bf*l_v) );
    suspension_jnts_["right"]->SetForce(0, -1*(kf*r_d)-(bf*r_v) );
  }

  private:
    std::map<std::string, physics::JointPtr> suspension_jnts_;
    double sus_kf_, sus_bf_;
    ros::Time last_update_time_;
    physics::ModelPtr model_;
    event::ConnectionPtr updateConnection_;
  };

  // Register plugin
  GZ_REGISTER_MODEL_PLUGIN(WarthogSuspensionPlugin);
};  // namespace gazebo
