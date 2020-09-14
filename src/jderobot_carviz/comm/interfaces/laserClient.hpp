#pragma once

#include </opt/jderobot/include/jderobot/types/laserData.h>


namespace Comm {

	/**
	 * @brief LaserClient class.
	 * This class is a Interface to seprate communications from tools. 
	 * With this, the tools don't need know which communicator (ROS or ICE) are using because both use the same interface.
	 *
	 */
	class LaserClient {
	public:
		virtual JdeRobotTypes::LaserData getLaserData() = 0;
		bool on = false;
	protected:
		JdeRobotTypes::LaserData laserData;
	};

} 
