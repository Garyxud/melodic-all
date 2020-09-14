/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef HAND_BRIDGE_HARDWARE_H_
#define HAND_BRIDGE_HARDWARE_H_

#include <boost/chrono.hpp>
#include "serial_port.h"

class HandBridgeHardware {
    boost::chrono::steady_clock::time_point start;
    SerialPort serial;

public:
    void init(char * param)
    {
        if(!serial.init(param)) exit(1);
    }
    unsigned long time()
    {
        return boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - start).count();
    }
    int read(){
        return serial.readByte();
    }
    void write(uint8_t* data, int length)
    {
        serial.write(data, length);
    }
};

#endif // HAND_BRIDGE_HARDWARE_H_
