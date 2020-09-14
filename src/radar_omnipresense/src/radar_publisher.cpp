/*Copyright 2018 Santa Clara University Robotic Systems Lab 
Licensed under the Educational Community License, Version 2.0 (the "License"); 
you may not use this file except in compliance with the License. 
You may obtain a copy of the License at

http://opensource.org/licenses/ECL-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
License for the specific language governing permissions and limitations under
the License.
*/

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "radar_omnipresense/radar_data.h"
#include "radar_omnipresense/SendAPICommand.h"
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream> 
#include <string>
#include <thread>
#include <vector>
#include "../lib/serialconnection/SerialConnection.h"
#include <cjson/cJSON.h>



SerialConnection * con;

/* Originally written to expect some combination of speed FFT  and raw data.  
* However, we're not going to support FFT or RAW at the moment, and even 
* if/when we do,  I and Q are in the same JSON message, so merge them down */
bool OJ = true;
bool OFft = false;
bool ORaw = false;

/*!
* \service function to send api commands to radar device
*
* This function allow the user to input an Omnipresense radar device API commands 'OF', or 'Of' to
* enable or disable the output of FFT data.
*
*/
bool api(radar_omnipresense::SendAPICommand::Request &req, radar_omnipresense::SendAPICommand::Response &res) 
{
  if (req.command == "Oj")
  {
    ROS_INFO("JSON mode required.  Wisely refusing to turn JSON mode off");
    return true;
  }
  if (req.command == "OR" || req.command == "OF")
  {
    ROS_INFO("FFT and RAw output are not currently supported.");
    return true;
  }

  //writes the api input to the serial port
  res.response = "false";
  con->write(std::string(req.command.c_str())); 
  con->clearBuffer();

  //logging output for user to see that correct input was sent to the radar
  ROS_INFO("API command sent: %s", req.command.c_str()); 
  con->waitForData(); // Hm, not sure why 2 waitForData's
  con->waitForData();
  std::string command_response = con->readString();
  ROS_INFO("Command_response is: %s", command_response.c_str());
  std::size_t found_open_brace = command_response.find("{");
  std::size_t found_close_brace = command_response.find("}");
  bool open_brace_missing = (found_open_brace == std::string::npos);
  bool close_brace_missing = (found_close_brace == std::string::npos);
  if ((open_brace_missing) && (close_brace_missing))
  {
    ROS_INFO("ERROR: Response not received");
    return true;
  }
  std::string whole_msg = command_response.substr(found_open_brace,found_close_brace+1);
  cJSON *json = cJSON_Parse(whole_msg.c_str()); 
  char *json_as_string = cJSON_Print(json);
  ROS_INFO("Received: %s", json_as_string);
  free(json);
  return true;
}


/*!
* \this function intakes the location of the msg struct, whole message, serial port and 
* then parses into the member fields of the struct.
*
* This function takes in the output of std::string getMessage(CommConnection *connection) 
* along with the object ptr of the package's message structure and the serialPort.
* It uses the JSON parser to populate the members of the package's custom message structure.
*
*/
void process_json(radar_omnipresense::radar_data *data, std::vector<std::string> msgs, std::string serialPort)
{
  for(int i = 0; i < msgs.size(); i++)
  {
    std::string single_msg = msgs[i];
    if (single_msg.empty())
    {
      continue;
    }
    cJSON *json = cJSON_Parse(single_msg.c_str()); 
    char *json_as_string = cJSON_Print(json);
    ROS_INFO("Received: %s", json_as_string);

    // some uncertainty about OutputFeature's formatting being bad.  Just in case....
    const cJSON *output_feature = cJSON_GetObjectItemCaseSensitive(json, "OutputFeature");
    if (output_feature != NULL) {
	ROS_INFO("OutputFeature received");
	free(json);
	return;
      }
    
    const cJSON *speed = cJSON_GetObjectItemCaseSensitive(json, "speed");
/* FFT parsing isn't required.
    bool fft = document.HasMember("FFT");
    bool raw_I = document.HasMember("I");
    bool raw_Q = document.HasMember("Q");
*/
    //parse speed, adapting to all API versions.  (value as string or number, optional 'direction') 
    if (speed != NULL)
    {
      //accesses the value for speed and assigns it to info.speed
      // deal with it whether it's a value (like pre 1.2) or string (1.2.0)
      if (speed->valuestring == NULL)
	data->speed = speed->valuedouble;
      else
        data->speed = atof(speed->valuestring);

      if (data->speed >= 0)
	data->direction = "inbound";
      else if (data->speed == 0)
	data->direction = "still";
      else
	data->direction = "outbound";
      // just in case a direction was received (v1.0), it should override sign
      const cJSON *direction = cJSON_GetObjectItemCaseSensitive(json, "direction");
      if (direction != NULL) {
	data->direction = direction->valuestring;
      }

      //accesses the numerical value for time and assigns it to info.time
      const cJSON *time = cJSON_GetObjectItemCaseSensitive(json, "time");
      if (time != NULL) {
	data->time = time->valuedouble;
        // in case it's an old 241 v 1.1
	const cJSON *tick = cJSON_GetObjectItemCaseSensitive(json, "tick");
        if (tick != NULL) {
            data->time += (float)tick->valueint/1000.0;
        }
      }

      //place holder for field sensorid.  For now, use port, and don't differentiate targets
      data->sensorid = serialPort; 
      data->objnum = 1;
      data->metadata.stamp = ros::Time::now(); 
    }

/* FFT not required yet
    //indexes and creates fft field for publishing.
    else if (fft)
    {
      for (int i = 0; i < document["FFT"].Size(); i++)
      {
        //FFT is an array of 1x2 array, each element represent a different channel. Either i or q.
         //TODO label as "i" is the real component and "q" is actually an Imaginary component.
        const Value& a = document["FFT"][i].GetArray();  
        data->fft_data.real.push_back(a[0].GetFloat());
        data->fft_data.imaginary.push_back(a[1].GetFloat());
       }  
    }
*/
    else 
    {
      ROS_INFO("Unsupported message type");
    }
    free(json);
  }
}

/*!
* \this function determines the number of report lines expected per actual 
* radar sample.  As only speed is implemented, this is going to be "1" 
*
* This functio determines the number of report lines expected per actual 
* radar sample.  As only speed is implemented, this is going to be "1"
*/
int get_msgs_filled() 
{
      bool msgs_filled[] = {OJ , OFft, ORaw};
      int ret_val = 0;
      for (int k = 0; k < sizeof(msgs_filled)/sizeof(msgs_filled[0]); k++)
      {
        if (msgs_filled[k])
        {
          ret_val++;
        }
      }
      return ret_val;
}
/*!
* \this function builds a message bit by bit from the serial port and checks to 
* make sure it is an appropriate message, if not it outputs and empty string.
*
* This function utilizes the LinuxCommConnection library to build a message that is 
* of complete JSON message format.  
* If 'speed' isn't within the JSON message it outputs an empty message 
* If more than one message was sent and they were not whole JSON messages 
* the function returns an empty string.
*
*/
std::vector<std::string> getMessage(SerialConnection *connection) 
{
  std::string msg;
  std::vector<std::string> msg_vec;
  bool startFilling = false;
  int num_expected_msgs = get_msgs_filled(); 
  for (int i = 0; i < num_expected_msgs; i++) 
  {
    msg = std::string();
    bool startFilling = false;
    bool check = false;
    while(true)
    {
      if(connection->available()) 
      {
        char c = connection->read();
        if(c == '{') 
        {
          if (startFilling)
          {
            msg = std::string();
            check = true;
            //return std::string();
          }
          startFilling = true;
        }
        else if(c == '}' && startFilling) 
        {
          msg += c;
          break;
        }
        if(!(check) && startFilling)
        {
          msg += c;
        }
      }
    }
    if (msg.find("speed") == std::string::npos && 
        msg.find("FFT") == std::string::npos && 
        msg.find("I") == std::string::npos && 
        msg.find("Q") == std::string::npos && 
        msg.find("OutputFeature") == std::string::npos)
    { 
      msg = std::string();
      msg_vec.push_back(msg); 
    }
    else
    {
      msg_vec.push_back(msg);
    }
  }
  return msg_vec;
}
//#########################################################################################################################################################//
//#########################################################################################################################################################//

int main(int argc, char** argv)
{
  //the name of this node: radar_publisher
  ros::init(argc, argv, "radar_publisher"); 
  ros::NodeHandle nh;//("~");

  std::string serialPort;
  // sets serialPort to "serialPort". "serialPort" is defined in package launch file. 
  // "/dev/ttyACM0" is the default value if launch does not set "serialPort" 
  // or launch is not used.
  nh.param<std::string>("serialPort", serialPort,"/dev/ttyACM0");

  //the node is created and publishes to topic "radar" using radar_omnipresense::radar_data 
  //messages and will buffer up to 1000 messages before beginning to throw away old ones.
  ros::Publisher radar_pub = nh.advertise<radar_omnipresense::radar_data>("radar_report",1000); 
  //the service "send_api_commands" is created and advertised over ROS
  ros::ServiceServer radar_srv = nh.advertiseService("send_api_command", api);  

  //ROS loop rate, currently sent to 60Hz.
  ros::Rate loop_rate(1000); 
  //Open USB port serial connection for two way communication
  SerialConnection connection = SerialConnection(serialPort.c_str(), B19200, 0);  
  con = &connection;
  //continues the while loop as long as ros::ok continues to continue true
  int is_initialized = 0;
  while (ros::ok())
  {
    if (connection.isConnected())
    {
      if (! is_initialized)
      {
        //assuming radar is being started with no fft output.
        //Then begin reading. Then set radar device to output Json format  data
        //string of format {"speed":#.##,"direction":"inbound(or outbound)","time":###,"tick":###}.   though actually, tick got appended to time so time is now in ms
        ROS_INFO("setting radar settings");
        connection.clearBuffer();
        connection.begin();
        connection.write("OJ");  // force JSON mode even for speed

        // Note: In 1.1 and earlier, seconds came in "time" and milliseconds in "tick"
        // In JSON, that was always present, and never present in 'non JSON'
        // The new command OT will do nothing in pre 1.2 and will turn on time
        // in either non-JSON or JSON nowadays.  Now, Time is in sec.millis
        // The latest JSON processing code will add tick to time (if found)
        // so that aligns both.
        connection.write("OT"); // In 1.2, this turns on "time" in sec.millis

        connection.write("Of"); // no FFT
        connection.write("Or"); // no raw
        connection.write("F2"); // 2 decimals
        connection.clearBuffer();
      }

      radar_omnipresense::radar_data info; 
      //creates an instance of the radar_data structure named info
      std::vector<std::string> msgs = getMessage(&connection);

      // ok, and this is the big action of this driver.  Parse it and send it.
      ROS_INFO("about to process incoming json message");
      process_json(&info, msgs, serialPort);
      radar_pub.publish(info);

      //becomes neccessary for subscriber callback functions
      ros::spinOnce();  
      // forces loop to wait for the remaining loop time to finish before starting over
      loop_rate.sleep();
      is_initialized=true;
    }
    else
    {
      ROS_INFO("Not Connected");
      connection.begin();
      is_initialized = 0;
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  ros::shutdown();
  return 0;
}
