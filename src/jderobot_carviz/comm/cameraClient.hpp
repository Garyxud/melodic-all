
#include </opt/jderobot/include/jderobot/types/image.h>


#include "communicator.hpp"
#include "interfaces/cameraClient.hpp"


#include "header/listenerCamera.hpp"





namespace Comm {

	/**
	 * @brief make a CameraClient using propierties
	 *
	 *
	 * @param communicator that contains properties
	 * @param prefix of client Propierties (example: "carViz.Camera")
	 * 
	 *
	 * @return null if propierties are wrong
	 */
	CameraClient* getCameraClient(Comm::Communicator* jdrc, std::string prefix);


}
