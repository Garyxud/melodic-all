
#pragma once
#include </opt/jderobot/include/jderobot/types/pose3d.h>

namespace Comm {

	/**
	 * @brief Pose3dClient class.
	 * This class is a Interface to seprate communications from tools. 
	 * With this, the tools don't need know which communicator (ROS or ICE) are using because both use the same interface.
	 *
	 */
	class Pose3dClient {
	public:
		virtual JdeRobotTypes::Pose3d getPose() = 0;
		bool on = false;
	protected:
		JdeRobotTypes::Pose3d pose;
	};

} 
