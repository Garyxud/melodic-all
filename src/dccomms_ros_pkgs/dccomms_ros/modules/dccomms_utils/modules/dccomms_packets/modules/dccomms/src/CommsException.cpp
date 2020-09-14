/*
 * RadioException.cpp
 *
 *  Created on: 06/03/2015
 *      Author: diego
 */

#include <cstring>
#include <dccomms/CommsException.h>
#include <string>

namespace dccomms {

CommsException::CommsException(std::string msg, int cod) {
  message = msg;
  code = cod;
}

CommsException::~CommsException() {}

} /* namespace radiotransmission */
