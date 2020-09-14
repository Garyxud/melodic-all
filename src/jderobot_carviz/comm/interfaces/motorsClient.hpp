#pragma once

#include </opt/jderobot/include/jderobot/types/cmdvel.h>

namespace Comm {

	/**
	 * @brief MotorsClient class.
	 * This class is a Interface to seprate communications from tools. 
	 * With this, the tools don't need know which communicator (ROS or ICE) are using because both use the same interface.
	 *
	 */
	class MotorsClient {
	public:
		virtual void sendVelocities(JdeRobotTypes::CMDVel vel ) = 0;
		bool on = false;
		virtual void sendVX (float vx) = 0;
		virtual void sendVY (float vy) = 0;
		virtual void sendAZ (float az) = 0;
		virtual void sendV (float v) = 0;
		virtual void sendW (float w) = 0;
		virtual void sendL (float l ) = 0;
	protected:
		JdeRobotTypes::CMDVel cmdvel;
	};

} 
