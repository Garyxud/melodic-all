#pragma once
#include <iostream>
#include </opt/jderobot/include/jderobot/types/image.h>
 
namespace camViz {
	class CameraClient {
	public:
		virtual JdeRobotTypes::Image getImage() = 0;
		virtual int getRefreshRate() = 0;
		bool on = false;
	protected:
		JdeRobotTypes::Image image;
		int refreshRate;
	};
}

