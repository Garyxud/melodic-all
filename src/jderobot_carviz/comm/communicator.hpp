#pragma once

#include "../config/properties.hpp"
//#include <jderobot/comm/tools.hpp>

namespace Comm {

	class Communicator {
	public:
		Communicator(Config::Properties config);
		~Communicator();

		Config::Properties getConfig();
		//Ice::CommunicatorPtr getIceComm();


	private:
		Config::Properties config;
		//Ice::CommunicatorPtr ic;	
	};


} 

