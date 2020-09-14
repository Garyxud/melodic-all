#include "laserClient.hpp"
#include <iostream>   // std::cout
#include <string>


namespace Comm {

LaserClient*
getLaserClient(Comm::Communicator* jdrc, std::string prefix){
	LaserClient* client = 0;
	int server;
	server = jdrc->getConfig().asInt(prefix+".Server");
	//server = std::stoi(server_name);

	switch (server){
		case 0:
		{
			std::cout << "Laser disabled" << std::endl;
			break;
		}
		case 1:
		{
            #ifdef ROS_H
                std::cout << "Receiving LaserData from ROS messages" << std::endl;
                std::string nodeName;
                nodeName =  jdrc->getConfig().asStringWithDefault(prefix+".Name", "LaserNode");
				std::string topic;
				topic = jdrc->getConfig().asStringWithDefault(prefix+".Topic", "");
                ListenerLaser* lc;
                lc = new ListenerLaser(0, nullptr, nodeName, topic);
                lc->start();
                client = (Comm::LaserClient*) lc;
            #else
                throw "ERROR: ROS is not available";
            #endif

		 	break;
		}
		case 2:
		{
            #ifdef ROS2_H
                std::cout << "Receiving LaserData from ROS messages" << std::endl;

            #else
                throw "ERROR: ROS2 is not available";
            #endif

		 	break;
		}
		default:
		{
			std::cerr << "Wrong " + prefix+".Server property" << std::endl;
			break;
		}

	}

	return client;


}

}
