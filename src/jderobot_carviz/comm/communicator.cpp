#include "communicator.hpp"

namespace Comm {

Communicator::Communicator(Config::Properties config){
	this->config = config;
	
	//this->ic = Ice::initialize();
}


Communicator::~Communicator(){
	//this->ic->destroy();
}


Config::Properties 
Communicator::getConfig(){
	return this->config;
}

//Ice::CommunicatorPtr 
//Communicator::getIceComm(){
//	return this->ic;
//}


}//NS
