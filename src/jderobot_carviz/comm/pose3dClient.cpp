#include "pose3dClient.hpp"

#include <iostream>   // std::cout
#include <string>

namespace Comm {

Pose3dClient*
getPose3dClient(Comm::Communicator* jdrc, std::string prefix){
	Pose3dClient* client = 0;
	int server;
	server = jdrc->getConfig().asInt(prefix+".Server");
	//server = std::stoi(server_name);

	switch (server){
		case 0:
		{
			std::cout << "Pose3d disabled" << std::endl;
			break;
		}
		case 1:
		{
			#ifdef ROS_H

				std::cout << "Receiving Pose3D from ROS interfaces" << std::endl;
				std::cout << "Receiving Pose3D from ROS messages" << std::endl;
				std::string nodeName;
				nodeName =  jdrc->getConfig().asStringWithDefault(prefix+".Name", "PoseNode");
				std::string topic;
				topic = jdrc->getConfig().asStringWithDefault(prefix+".Topic", "");
				ListenerPose* lc;
				lc = new ListenerPose(0, nullptr, nodeName, topic);
				lc->start();
				client = (Comm::Pose3dClient*) lc;

            #else
			throw "ERROR: ROS is not available";
			#endif
		 	break;

		}
		case 2:
		{
			#ifdef ROS2_H
				std::cout << "Receiving Pose3D from ROS2 interfaces" << std::endl;
            #else
				throw "ERROR: ROS2 is not available";
			#endif
		 	break;
		}
		default:
		{
			std::cerr << "Wrong version chosen" << std::endl;
			break;
		}

	}

	return client;


}

}
