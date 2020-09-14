#ifndef LEUZE_PHIDGET_DRIVER_H
#define LEUZE_PHIDGET_DRIVER_H

#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "leuze_msgs/PhidgetIKInputMsg.h"
#include "leuze_msgs/PhidgetIKOutputMsg.h"

class LeuzePhidgetDriver
{
public:
  LeuzePhidgetDriver(ros::NodeHandle* nh);

protected:
  void readInputStateCallback(const std_msgs::BoolConstPtr &msg, int i);
  void getOutputStateCallback(const leuze_msgs::PhidgetIKOutputMsgConstPtr &msg);
  void spawnInputSubscribers();
  void spawnOutputPublishers();
  void publishInputState();
  void publishOutputState();


private:
  ros::NodeHandle nh_;
  ros::Publisher pub_show_inputs_;
  ros::Subscriber sub_read_input_;
  ros::Subscriber sub_get_outputs_;
  std::vector<int> input_state_;
  std::vector<int> output_state_;
  std::vector<ros::Subscriber> input_subscribers_;
  std::vector<ros::Publisher> output_publishers_;
  const int _NUMBER_OF_IK_INPUTS_ = 11;
  const int _NUMBER_OF_IK_OUTPUTS_= 14;
};

#endif // LEUZE_PHIDGET_DRIVER_H
