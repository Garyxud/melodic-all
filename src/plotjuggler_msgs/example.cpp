#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <cmath>
#include "pj_msgs/msg/dictionary.hpp"
#include "pj_msgs/msg/data_points.hpp"

using namespace std::chrono_literals;

pj_msgs::msg::DataPoint CreateDataPoint(uint16_t name_index, double time, double value)
{
  pj_msgs::msg::DataPoint point;
  point.stamp = time;
  point.name_index = name_index;
  point.value = value;
  return point;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pj_test");

  auto publisher_dict = node->create_publisher<pj_msgs::msg::Dictionary>("pj_msg_dictionary",
                                                                         rclcpp::QoS(10).transient_local());
  auto publisher = node->create_publisher<pj_msgs::msg::DataPoints>("pj_msg_data", 10);

  std::random_device rd;
  const uint32_t UUID = rd(); // just a random number

  pj_msgs::msg::Dictionary dictionary;

  dictionary.dictionary_uuid = UUID;
  dictionary.names.push_back("sensor_a"); // index 0
  dictionary.names.push_back("sensor_b"); // index 1
  dictionary.names.push_back("sensor_c"); // index 2

  publisher_dict->publish(dictionary);


  rclcpp::WallRate loop_rate(50ms);
  double t=0;
  while (rclcpp::ok()) {

    t += 0.1;
    pj_msgs::msg::DataPoints msg;
    msg.dictionary_uuid = UUID;
    msg.samples.push_back( CreateDataPoint(0, t, std::sin(t)));
    msg.samples.push_back( CreateDataPoint(1, t, std::cos(t)));
    msg.samples.push_back( CreateDataPoint(2, t, 2*std::cos(t)));

    publisher->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
