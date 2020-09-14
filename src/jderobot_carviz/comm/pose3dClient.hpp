#include </opt/jderobot/include/jderobot/types/laserData.h>
#include "communicator.hpp"
#include "interfaces/pose3dClient.hpp"
#include "header/listenerPose.hpp"

namespace Comm {

	/**
	 * @brief make a Pose3dClient using propierties
	 *
	 *
	 * @param communicator that contains properties
	 * @param prefix of client Propierties (example: "carViz.Pose3d")
	 * 
	 *
	 * @return null if propierties are wrong
	 */
	Pose3dClient* getPose3dClient(Comm::Communicator* jdrc, std::string prefix);

} 
