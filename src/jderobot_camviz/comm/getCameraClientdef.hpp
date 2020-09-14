#include <iostream>
#include "sstream"
#include <chrono>
#include <memory>

#include "mainsub.hpp"
#include "getCameraClient.hpp"


namespace camViz{

	CameraClient* 
	getCameraClient(int argc, char** argv, int server, std::string topic,std::string  nodeName){
		CameraClient* client = NULL;		
		switch (server){
			case 1:
			{
				#ifdef ROS1S_H
				std::cout << "Receiving ROS1 messages" << std::endl;
	
				ListenerCamera* lc;
				lc = new ListenerCamera(0, nullptr, nodeName, topic);
				lc->start();

				client = (camViz::CameraClient*) lc;
				//cv::destroyWindow("view");
					
				#else
					throw "ERROR: ROS1 is not available";
				#endif
				break;
			}
			case 2:
			{
				std::cout << "This version supports only ROS1 messages" << std::endl;
	
				break;
			}
			default:
			{
				printf("Select right distro of ROS");
				break;
			}

		}
	
		return client;


	}

}
