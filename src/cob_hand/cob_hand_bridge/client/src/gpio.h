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


#ifndef HAND_BRIDGE_GPIO_H_
#define HAND_BRIDGE_GPIO_H_

#include "pigpio.h"

class GPIO {
    bool initialized;
    uint32_t out_mask0;
    bool call(int (*func)(uint32_t), uint32_t mask, uint32_t pins){
        return initialized && (pins & mask) == pins && (pins == 0 || func(pins) == 0);
    }
public:
    GPIO() : initialized(false), out_mask0(0) {}
    bool isInitialized() const { return initialized; }
    bool setInput(uint32_t pin) {
        return initialized && gpioSetMode(pin, PI_INPUT) == 0;
    }
    bool setOutput(uint32_t pin) {
        if(initialized && gpioSetMode(pin, PI_OUTPUT) == 0){
            if(pin < 32) out_mask0 |= (1<<pin);
            return true;
        }
        return false;
    }
    bool init(){
        return initialized = gpioInitialise() >= 0;
    }
    bool clearPins(uint32_t pins){
        return call(gpioWrite_Bits_0_31_Clear, out_mask0, pins);
    }
    bool setPins(uint32_t pins){
        return call(gpioWrite_Bits_0_31_Set, out_mask0, pins);
    }
    bool writePin(uint32_t pin, uint32_t level) {
        if(!initialized) return false;
        if(pin < 32 &&  ((1<<pin) & out_mask0) != (1<<pin)) return false;

        return gpioWrite(pin, level ? 1 : 0) == 0;
    }
    bool pwmPin(uint32_t pin, float level) {
        if(!initialized) return false;
        if(pin >= 32 ||  ((1<<pin) & out_mask0) != (1<<pin)) return false;

        int range;
        if ((range = gpioGetPWMrange(pin)) < 0) return false;

        if(level < 0 || level > 1.0) return false;

        return gpioPWM(pin, level* range) == 0;
    }
    uint32_t getState() {
        return initialized ? gpioRead_Bits_0_31(): 0;
    }
};
#endif // HAND_BRIDGE_GPIO_H_
