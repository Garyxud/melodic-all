/// Copyright 2019 Continental AG
///
/// @file camera_commander.cpp
///
/// @brief This file implements the CameraCommander class methods.
///
#include "camera_commander/camera_commander.h"
#include <pluginlib/class_list_macros.h>
#include "image_processor/hfl110dcu.h"
#include <string>
#include <vector>

namespace hfl
{
CameraCommander::CameraCommander()
{
  /// Initialize default variables here
}

CameraCommander::~CameraCommander()
{
  // Stop camera if active
  if (current_state_ != state_probe)
  {
    ROS_INFO("Shutting down camera...");
  }
}

void CameraCommander::onInit()
{
  // Initialize node handler
  node_handler_ = getPrivateNodeHandle();
  namespace_ = node_handler_.getNamespace();

  // Initialize flash object (pointer)
  if (!setFlash())
  {
    throw - 1;
  }
  // Initialize UPD services, sockets, and subscribers
  if (!udpInit())
  {
    throw - 1;
  }
  // Initialize current state and timer_ callback
  current_state_ = state_probe;
  previous_state_ = state_probe;
  auto set_state_callback = std::bind(&CameraCommander::setCommanderState, this, std::placeholders::_1);
  timer_ = node_handler_.createTimer(ros::Duration(1), set_state_callback);
}

bool CameraCommander::createSocket(std::string computer_addr, std::string camera_addr, uint16_t port, bool isMulticast)
{
  // Create a socket Request service message
  udp_com::UdpSocket socket_request;
  // Populate request service message
  socket_request.request.srcAddress = computer_addr;
  socket_request.request.destAddress = camera_addr;
  socket_request.request.port = port;
  socket_request.request.isMulticast = isMulticast;
  // Return service call response
  if (ros::service::waitForService(udp_socket_creation_service_client_.getService(), -1) &&
      udp_socket_creation_service_client_.call(socket_request))
  {
    return true;
  }
  // Return false if service call failed
  return false;
}

bool CameraCommander::udpInit()
{
  // Get ethernet interface
  node_handler_.getParam("ethernet_interface", ethernet_interface_);
  ROS_INFO("%s/ethernet_interface: %s", node_handler_.getNamespace().c_str(), ethernet_interface_.c_str());

  // Get camera IP address
  node_handler_.getParam("camera_ip_address", camera_address_);
  ROS_INFO("%s/camera_ip_address:      %s", namespace_.c_str(), camera_address_.c_str());

  // Get computer IP address
  node_handler_.getParam("computer_ip_address", computer_address_);
  ROS_INFO("%s/computer_ip_address:      %s", namespace_.c_str(), computer_address_.c_str());

  // Get frame data port number
  node_handler_.getParam("frame_data_port", frame_data_port_);
  ROS_INFO("%s/frame_data_port:      %i", namespace_.c_str(), frame_data_port_);

  // Get ethernet namespace node handler
  ros::NodeHandle ethernet_interface_handler(ethernet_interface_);

  // Initialize udp Create Socket service client
  udp_socket_creation_service_client_ = ethernet_interface_handler.serviceClient<udp_com::UdpSocket>("udp/"
                                                                                                     "create_socket");
  // Initialize udp send service client
  udp_send_service_client_ = ethernet_interface_handler.serviceClient<udp_com::UdpSend>("udp/send");

  ROS_INFO("Checking for UDP Communication...");
  ros::service::waitForService(udp_socket_creation_service_client_.getService(), -1);
  ros::service::waitForService(udp_send_service_client_.getService(), -1);
  ROS_INFO("UDP Communication online");

  // Create a Frame_Data Socket
  if (!createSocket(computer_address_, camera_address_, frame_data_port_, false))
  {
    // Socket Not Created!
    return false;
  }

  // Subscribe to Frame_Data Socket
  frame_data_subscriber_ =
    ethernet_interface_handler.subscribe(std::string("udp/p") +
       std::to_string(frame_data_port_), 1000,
       &CameraCommander::frameDataCallback, this);
  // Everything Initialized
  return true;
}

bool CameraCommander::setFlash()
{
  // Parameter temporal variables
  std::string model, version, frame_id;

  // Get camera model
  node_handler_.getParam("model", model);
  ROS_INFO("%s/model:             %s", namespace_.c_str(), model.c_str());
  // Get camera version
  node_handler_.getParam("version", version);
  ROS_INFO("%s/version:           %s", namespace_.c_str(), version.c_str());
  // Get camera frame_id
  node_handler_.getParam("frame_id", frame_id);
  ROS_INFO("%s/frame_id:          %s", namespace_.c_str(), frame_id.c_str());

  // Initialize flash object
  try
  {
    // Load HFL class instance
    if (model == "hfl110dcu")
    {
      flash_.reset(new HFL110DCU(model, version, frame_id, node_handler_));
    }
    else
      ROS_ERROR("Camera model not found!");
    }
  catch (int e)
  {
    ROS_ERROR("Camera load failed!");
    return false;
  }

  // Return
  return true;
}

void CameraCommander::setCommanderState(const ros::TimerEvent&)
{
  std::vector<uint8_t> start_command =
  {
    0x1C, 0x00
  };
  // Executes states accordingly
  switch (current_state_)
  {
    // Establishes connection
    case state_probe:
      ROS_INFO_ONCE("Establishing connection...");
      break;
    // Sets camera registers
    case state_init:
      // Update Commander state
      previous_state_ = state_probe;
      current_state_ = state_done;
      ROS_INFO("Camera active");

      if (!dynamic_parameters_server_)
      {
        // Initialize dynamic reconfigure server
        dynamic_parameters_server_ =
          std::make_shared<dynamic_reconfigure::Server<hfl_driver::HFLConfig> >(
            node_handler_);
        // set dynamic reconfigure callback
        dynamic_reconfigure::Server<hfl_driver::HFLConfig>::CallbackType
          dynamic_parameters_callback =
            boost::bind(&CameraCommander::dynamicPametersCallback, this, _1, _2);
        dynamic_parameters_server_->setCallback(dynamic_parameters_callback);
      }

      break;
    // Done camera
    case state_done:
      error_status_ = checkForError();
      if (error_status_ != no_error)
      {
        previous_state_ = state_done;
        current_state_ = state_error;
      }
      break;

    case state_error:
      if (fixError(error_status_))
      {
        current_state_ = previous_state_;
      }
      break;
    // Default state
    default:
      // Restart
      current_state_ = state_probe;
      previous_state_ = state_probe;
      break;
  }
}

error_codes CameraCommander::checkForError()
{
  // Check for Frame Data Publisher on frame data topic
  if (frame_data_subscriber_ != NULL && frame_data_subscriber_.getNumPublishers() <= 0)
  {
    return frame_socket_error;
  }

  // Check for Object Data Publisher on object data topic
  if (object_data_subscriber_ != NULL && object_data_subscriber_.getNumPublishers() <= 0)
  {
    return object_socket_error;
  }

  // Check for LUT Data Publisher on lut data topic
  if (lut_data_subscriber_ != NULL && lut_data_subscriber_.getNumPublishers() <= 0)
  {
    return lut_socket_error;
  }
}

bool CameraCommander::fixError(error_codes error)
{
  switch (error)
  {
    case frame_socket_error:
      // Create Frame Socket
      return createSocket(computer_address_, camera_address_, frame_data_port_, false);
      break;
    case no_error:
      // Return true
      return true;
      break;
    default:
      // Not sure what is wrong.. Go back to state_done to check for error..
      return true;
  };
}

void CameraCommander::frameDataCallback(const udp_com::UdpPacket& udp_packet)
{
  // Checks UPD package source IP address
  if (udp_packet.address == camera_address_)
  {
    switch (current_state_)
    {
      case state_probe:
        ROS_INFO_ONCE("Connection established with Frame Data UDP Port!");
        previous_state_ = state_probe;
        current_state_ = state_init;
        break;
      case state_done:
        ROS_INFO_ONCE("Frame Data UDP packages arriving...");
        flash_->processFrameData(udp_packet.data);
        break;
    }
  }
}

bool CameraCommander::sendCommand(const std::vector<uint8_t>& data)
{
  // Create a UDP request service message
  udp_com::UdpSend send_request;
  // Populate request service message
  send_request.request.address = computer_address_;
  send_request.request.srcPort = frame_data_port_;
  send_request.request.dstPort = frame_data_port_;
  send_request.request.data = data;

  // Return service call response
  if (ros::service::exists(udp_send_service_client_.getService(), false) &&
      udp_send_service_client_.call(send_request))
  {
    return send_request.response.sent;  // Will always be true
  }
  else if (!send_request.response.socketCreated)
  {
    // response.socketCreated will be false right at start up
    // error is immediately set since sockets were not created
    current_state_ = state_error;
    error_status_ = frame_socket_error;
  }
  else
  {
    ROS_ERROR("Could not send data to sensor");
    ROS_INFO("Please check the connections to the sensor. ");
  }

  // Return false if service call failed
  return false;
}

void CameraCommander::dynamicPametersCallback(hfl_driver::HFLConfig& config, uint32_t level)
{
  // Checks if camera active
  if (current_state_ != state_done)
  {
    return;
  }
}
}  // end of namespace hfl
PLUGINLIB_EXPORT_CLASS(hfl::CameraCommander, nodelet::Nodelet)
