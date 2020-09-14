#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import shutil
import rospkg
import rosserial_client
from rosserial_client.make_library import *


def write_header(output_path, package, msg):
    output_path = output_path + "/" + package
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    with open(output_path + "/" + msg.name + ".h", "w") as header:
        msg.make_header(header)

def make_message(rospack, package, message, dest):
    path = rospack.get_path(package) + "/msg/" + message + ".msg"
    with open(path) as f:
        definition = f.readlines()
    md5sum = roslib.message.get_message_class(package+'/'+message)._md5sum
    write_header(dest, package, Message(message, package, definition, md5sum))

def make_service(rospack, package, service, dest):
    path = rospack.get_path(package) + "/srv/" + service + ".srv"
    with open(path) as f:
        definition = f.readlines()
    md5req = roslib.message.get_service_class(package+'/'+service)._request_class._md5sum
    md5res = roslib.message.get_service_class(package+'/'+service)._response_class._md5sum
    write_header(dest, package, Service(service, package, definition, md5req, md5res ) )


ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          4, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('double',            8, PrimitiveDataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}

rospack = rospkg.RosPack()
rospack.list = lambda: []
rosserial_generate(rospack, "ros_lib", ROS_TO_EMBEDDED_TYPES)

shutil.rmtree("ros_lib", True)
shutil.copytree(rospack.get_path("rosserial_embeddedlinux")+"/src/ros_lib", "ros_lib")

rosserial_client_copy_files(rospack, "ros_lib/")

make_message(rospack, "std_msgs", "Time", "ros_lib")
make_message(rospack, "std_msgs", "UInt8", "ros_lib")
make_service(rospack, "std_srvs", "Trigger", "ros_lib")

MakeLibrary("rosserial_msgs", "ros_lib", rospack)
MakeLibrary("cob_hand_bridge", "ros_lib", rospack)
