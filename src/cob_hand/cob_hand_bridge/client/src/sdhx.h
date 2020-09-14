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


#ifndef HAND_BRIDGE_SDHX_H_
#define HAND_BRIDGE_SDHX_H_

#include "serial_port.h"
#include "boost/format.hpp"
#include "boost/thread.hpp"

class SDHX {
    SerialPort serial;
    uint8_t rc;
    volatile bool initialized, reading;
    boost::mutex send_mutex;
    boost::mutex data_mutex;
    boost::thread read_thread;
    boost::chrono::steady_clock::time_point last_time;

    void doRead(){
        const size_t MAX_LINE = 1024;

        char buffer[MAX_LINE+1] = {0}; // extra byte for termination

        int a = 0, r = 1, offset = 0;
        while(a >= 0 && r > 0){
            if((a = serial.waitData(boost::chrono::milliseconds(1))) > 0 &&  (r = serial.read(buffer+offset, MAX_LINE-offset)) > 0){
                char * line = buffer;
                while(char * extra = strchr(line, '\n')){
                    *extra=0;
                    switch(line[0]){
                        case 'r': tryParseRC(line); break;
                        case 'P': tryReadValues(line, pos, "P=%hd,%hd", true); break;
                        case 'V': tryReadValues(line, vel, "V=%hd,%hd"); break;
                        case 'C': tryReadValues(line, cur, "C=%hd,%hd"); break;
                    }
                    line = extra+1;
                }
                offset = (buffer + offset + r - line);
                strncpy(buffer, line, offset);
            }
        }
        std::cerr << "stopped reading" << std::endl;
        boost::mutex::scoped_lock lock(data_mutex);
        reading = false;
    }

    bool tryParseRC(const char *line){
        uint8_t res;
        if(sscanf(line, "rc=%hhx", &res) == 1){
            if(res != 0){
                boost::mutex::scoped_lock lock(data_mutex);
                rc = res;
            }
            return true;
        }
        return false;
    }
    template<typename T> bool tryReadValues(const char *line, T (&val)[2], const char *format, bool track_time = false) {
        T val1,val2;
        if(sscanf(line, format, &val1, &val2) == 2){
            boost::mutex::scoped_lock lock(data_mutex);
            val[0] = val1;
            val[1] = val2;
            if(track_time){
                //std::cout << "rate: " <<  1.0/boost::chrono::duration_cast<boost::chrono::duration<double> >(boost::chrono::steady_clock::now() - last_time).count() << std::endl;
                last_time = boost::chrono::steady_clock::now();
            }
            return true;
        }
        return false;
    }

    bool send(const std::string &command){
        boost::mutex::scoped_lock lock(send_mutex);
        return serial.write(command);
    }

    bool setPWM(const int16_t &val, const char *format) {
        return val == 0 || send(boost::str(boost::format(format) % val));
    }

    int16_t pos[2], vel[2], cur[2];
public:
    SDHX() : rc(0), initialized(false), reading(false) {
    }
    ~SDHX(){
        if(initialized && read_thread.joinable()){
            read_thread.interrupt();
            read_thread.join();
        }
    }
    bool init(const char* port, int16_t min_pwm0, int16_t min_pwm1, int16_t max_pwm0, int16_t max_pwm1){
        std::string dev(port);
        if(!serial.isOpen() && !serial.init(&dev[0])) return false;

        boost::mutex::scoped_lock lock(data_mutex);
        if(!reading){
            reading = true;
            read_thread = boost::thread(boost::bind(&SDHX::doRead, this));
        }

        lock.unlock();
        if(setPWM(min_pwm0, "set min_pwm0 %1%\r\n") &&
                setPWM(min_pwm1, "set min_pwm1 %1%\r\n") &&
                setPWM(max_pwm0, "set max_pwm0 %1%\r\n") &&
                setPWM(max_pwm1, "set max_pwm1 %1%\r\n")){
            lock.lock();
            rc = 0;
            initialized = true;
            return true;
        }
        return false;
    }
    bool isInitialized() {
        return initialized;
    }

    template <typename Dur> bool getData(int16_t (&p)[2], int16_t (&v)[2], int16_t (&c)[2], const Dur &max_age){
        boost::mutex::scoped_lock lock(data_mutex);
        if((boost::chrono::steady_clock::now() - last_time) > max_age) return false;
        if(!reading) return false;

        p[0] = pos[0];
        p[1] = pos[1];
        v[0] = vel[0];
        v[1] = vel[1];
        c[0] = cur[0];
        c[1] = cur[1];

        return true;
    }
    bool poll(){
         return initialized && send("p;v;c\r\n");
    }
    bool halt(){
        return initialized && send("s\r\n");
    }
    bool move(const int16_t (&p)[2], const int16_t (&v)[2], const int16_t (&c)[2]){
        static boost::format command("m %hd,%hd %hd,%hd %hd,%hd\r\n");
        return initialized && send(boost::str(command % p[0] % p[1] % v[0] % v[1] % c[0] % c[1]));
    }
/*    bool move(const int16_t (&p)[2], const int16_t (&v)[2], const int16_t (&c)[2]){
        static boost::format command("m %hd,%hd\r\n");
        return initialized && send(boost::str(command % p[0] % p[1]));
    }
*/
    uint8_t getRC(){
        boost::mutex::scoped_lock lock(data_mutex);
        return rc;
    };
};
#endif // HAND_BRIDGE_SDHX_H_
