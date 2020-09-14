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


import time
import inspect

import rospy

from cob_android_msgs.srv import Script, ScriptResponse
from simple_script_server import simple_script_server  ## pylint: disable=no-name-in-module
sss = simple_script_server()

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class script_server():
	## Initializes the actionlib interface of the script server.
	#
	def __init__(self):
		rospy.Service('/script_server_android/script_service', Script, self.service_cb)

	def service_cb(self, req):
		res = ScriptResponse()
		if req.function_name == "trigger":
			if req.parameter_name == "init":
				handle01 = sss.init(req.component_name, blocking=False)
			elif req.parameter_name == "stop":
				handle01 = sss.stop(req.component_name, mode=req.mode, blocking=False)
			elif req.parameter_name == "recover":
				handle01 = sss.recover(req.component_name, blocking=False)
			elif req.parameter_name == "halt":
				handle01 = sss.halt(req.component_name, blocking=False)
			elif req.parameter_name == "compose_trajectory":
				handle01 = sss.compose_trajectory(req.component_name, req.parameter_name)
			else:
				handle01 = sss.trigger(req.component_name, req.parameter_name, blocking=False)
		elif req.function_name == "move":
			handle01 = sss.move(req.component_name,req.parameter_name, blocking=False, mode=req.mode)
		elif req.function_name == "move_base_rel":
			handle01 = sss.move_base_rel(req.component_name,req. parameter_name, blocking=False)
		elif req.function_name == "light":
			handle01 = sss.set_light(req.component_name, req.parameter_name, blocking=False)
		elif req.function_name == "stop":
			handle01 = sss.stop(req.component_name, blocking=False)
		elif req.function_name == "init":
			handle01 = sss.init(req.component_name, blocking=False)
		elif req.function_name == "recover":
			handle01 = sss.recover(req.component_name, blocking=False)
		elif req.function_name == "halt":
			handle01 = sss.halt(req.component_name, blocking=False)
		elif req.function_name == "compose_trajectory":
			handle01 = sss.compose_trajectory(req.component_name, req.parameter_name)
		elif req.function_name == "trigger_action":
			handle01 = sss.trigger_action(req.component_name, req.parameter_name)
		else:
				rospy.logerr("function <<%s>> not supported", req.function_name)
				res.error_code = -1
				return res

		res.error_code = handle01.get_error_code()
		if res.error_code == 0:
			rospy.logdebug("service result success")
		else:
			rospy.logerr("service result error")
		return res

## Main routine for running the script server
#
def script_server_android_main():
	rospy.init_node('script_server')
	script_server()
	rospy.loginfo("script_server is running")
	rospy.spin()
