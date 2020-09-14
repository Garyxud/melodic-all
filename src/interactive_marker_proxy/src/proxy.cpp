/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_client.h>
#include <interactive_marker_proxy/GetInit.h>

using namespace interactive_markers;

class Proxy
{
public:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  tf::TransformListener tf_;
  interactive_markers::InteractiveMarkerClient client_;
  std::string topic_ns_;
  std::string target_frame_;
  ros::ServiceServer service_;
  std::map<std::string, std::string> status_text_;
  ros::Timer timer_;

  std::map<std::string, visualization_msgs::InteractiveMarker> int_markers_;

  Proxy(std::string target_frame, std::string topic_ns) :
      client_(tf_, target_frame, topic_ns), topic_ns_(topic_ns), target_frame_(target_frame)
  {
    ROS_INFO_STREAM("Subscribing to " << topic_ns);
    ROS_INFO_STREAM("Target frame set to " << target_frame);

    client_.setInitCb(boost::bind(&Proxy::initCb, this, _1));
    client_.setUpdateCb(boost::bind(&Proxy::updateCb, this, _1));
    client_.setResetCb(boost::bind(&Proxy::resetCb, this, _1));
    client_.setStatusCb(boost::bind(&Proxy::statusCb, this, _1, _2, _3));
    client_.subscribe(topic_ns_);

    pub_ = nh_.advertise<visualization_msgs::InteractiveMarkerUpdate>(topic_ns_ + "/tunneled/update", 1000);

    service_ = nh_.advertiseService(topic_ns_ + "/tunneled/get_init", &Proxy::getInit, this);

    ros::NodeHandle private_nh("~");
    double update_rate;
    private_nh.param<double>("update_rate", update_rate, 30.0f);
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), boost::bind(&Proxy::timerCb, this, _1));
  }

  typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdatePtr UpdatePtr;

  void timerCb(const ros::TimerEvent&)
  {
    client_.update();
  }

  bool getInit(interactive_marker_proxy::GetInit::Request& request,
               interactive_marker_proxy::GetInit::Response& response)
  {
    ROS_INFO("Init requested.");
    std::vector< visualization_msgs::InteractiveMarker > markers;
    std::map<std::string, visualization_msgs::InteractiveMarker>::iterator it;
    for( it = int_markers_.begin(); it!=int_markers_.end(); it++ )
    {
      response.msg.markers.push_back(it->second);
    }
    return true;
  }

  void updateCb(const UpdateConstPtr& up_msg)
  {
    const visualization_msgs::InteractiveMarkerUpdate::_erases_type& erases = up_msg->erases;
    for (unsigned i = 0; i < erases.size(); i++)
    {
      int_markers_.erase(erases[i]);
    }

    const visualization_msgs::InteractiveMarkerUpdate::_poses_type& poses = up_msg->poses;
    for (unsigned i = 0; i < poses.size(); i++)
    {
      int_markers_[poses[i].name].pose = poses[i].pose;
      int_markers_[poses[i].name].header = poses[i].header;
    }

    const visualization_msgs::InteractiveMarkerUpdate::_markers_type& markers = up_msg->markers;
    for (unsigned i = 0; i < markers.size(); i++)
    {
      int_markers_[markers[i].name] = markers[i];
    }

    pub_.publish(up_msg);
  }

  void initCb(const InitConstPtr& init_msg)
  {
    UpdatePtr update(new visualization_msgs::InteractiveMarkerUpdate());
    update->markers = init_msg->markers;
    update->seq_num = init_msg->seq_num;
    update->server_id = init_msg->server_id;

    int_markers_.clear();
    updateCb(update);
  }

  void statusCb(InteractiveMarkerClient::StatusT status, const std::string& server_id, const std::string& status_text)
  {
    if ( status_text_.find(server_id) != status_text_.end() &&
         status_text_[server_id] == status_text )
    {
      return;
    }
    status_text_[server_id] = status_text;
    std::string status_string[] = {"INFO", "WARN", "ERROR"};
    ROS_INFO_STREAM( "(" << status_string[(unsigned)status] << ") " << server_id << ": " << status_text);
  }

  void resetCb(const std::string& server_id)
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactive_marker_proxy");
  {
    ros::NodeHandle nh;
    Proxy proxy(nh.resolveName("target_frame"), nh.resolveName("topic_ns"));
    ros::spin();
  }
}
