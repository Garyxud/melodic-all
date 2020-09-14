#include "motorsClient.hpp"
#include <iostream>   // std::cout
#include <string>

namespace Comm {

MotorsClient*
getMotorsClient(Comm::Communicator* jdrc, std::string prefix){
	MotorsClient* client = 0;
	int server;
	server = jdrc->getConfig().asInt(prefix+".Server");
	//server = std::stoi(server_name);

	switch (server){
		case 0:
		{
			std::cout << "Motors disabled" << std::endl;
			break;
		}
		case 1:
		{
			#ifdef ROS_H

				std::cout << "Sending Velocities by ROS messages" << std::endl;
				std::string nodeName;
				nodeName =  jdrc->getConfig().asStringWithDefault(prefix+".Name", "MotorsNode");
				std::string topic;
				topic = jdrc->getConfig().asStringWithDefault(prefix+".Topic", "");
				PublisherMotors* pm;
				pm = new PublisherMotors(0, nullptr, nodeName, topic);
				pm->start();
				client = (Comm::MotorsClient*) pm;
            #else
				throw "ERROR: ROS is not available";
			#endif
		 	break;
		}
		case 2:
		{
			#ifdef ROS2_H
				std::cout << "Sending Velocities by ROS2 messages" << std::endl;
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
