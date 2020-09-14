
#include </opt/jderobot/include/jderobot/types/laserData.h>
#include "communicator.hpp"
#include "interfaces/laserClient.hpp"
#include "header/listenerLaser.hpp"

#include <iostream>

//using namespace std;

namespace Comm {

	/**
	 * @brief make a LaserClient using properties
	 *
	 *
	 * @param communicator that contains properties
	 * @param prefix of client Propierties (example: "carViz.Laser")
	 * 
	 *
	 * @return null if propierties are wrong
	 */
	LaserClient* getLaserClient(Comm::Communicator* jdrc, std::string prefix);


} 
