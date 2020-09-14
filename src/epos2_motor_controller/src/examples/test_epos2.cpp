/*
Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Martí Morta Garriga  (mmorta@iri.upc.edu)
All rights reserved.

This file is part of IRI EPOS2 Driver
IRI EPOS2 Driver is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include "Epos2.h"

using namespace std;

/**
 * \example test_epos2.cpp
 *
 * This is an example on how to use epos2
 *
 * This example shows how to move a motor in a specified velocity.
 * It initates a new epos2 object called controller, checks if exist any error
 * (always a CAN passive mode error exists at switch on the controller). Then it
 * tries to enable motor in velocity mode and moves it 5 seconds at 300 rpm in
 * motor. After this time, it deletes the object then epos2 is disconnected.
 *
 * This should be the output of the example
 * \verbatim
 * $ sudo ./bin/test_epos2
 * Epos2 Example:
 * <Epos2 hardware state> RED LED on (if just plugged power in)
 * [FTDI] ftdi_read_chipid: 0
 * [FTDI] chipid: 667934CD
 * [EPOS2] online
 * 	RX No Data
 * 	: Resource temporarily unavailable
 * [EPOS2] State: 1
 * [EPOS2] to Fault Reset
 * [EPOS2] State: 3
 * [EPOS2] to Shutdown
 * [EPOS2] State: 4
 * [EPOS2] to Switch on
 * [EPOS2] State: 5
 * [EPOS2] to Enable Operation
 * [EPOS2] State: 8
 * [EPOS2] Motor Ready
 * [EPOS2] Operation Mode: -2
 * <Epos2 hardware state> GREEN LED on
 * [EPOS2] Mode Velocity Started
 * Wait 5 seconds
 * [EPOS2] Mode Velocity Stopped
 *
 * [EPOS2] offline
 * <Epos2 hardware state> Green Led blinking
 * Example ended
 * \endverbatim
 */

int main(void){

	CEpos2 controller;

  cout << "\n EPOS2 EXAMPLE" << endl;


	try{

    // Initiate an EPOS 2 controller
    controller.setVerbose(true);

    cout << "\n  Example : Init\n" << endl;
    controller.init();

    // enable motor in velocity mode
    cout << "\n  Example : Enable Controller\n" << endl;
    controller.enableController();
    cout << "\n  Example : GREEN LED blinking (controller enabled)\n" << endl;
    sleep(2);

    cout << "\n  Example : Enable Motor\n" << endl;
    controller.enableMotor(controller.VELOCITY);
    cout << "\n  Example : GREEN LED on (motor enabled)\n" << endl;

    controller.setOperationMode(controller.VELOCITY);
    cout << "\n  Example : Current Position = " << controller.readPosition() << " [qc]" << endl;

    // set motor velocity in RPM
    cout << "\n  Example : Velocity (500 1/min) during 2 seconds\n" << endl;
    controller.setTargetVelocity(500);

    // start the movement
    controller.startVelocity();
    sleep(2);

    // stop the movement
    controller.stopVelocity();

    cout << "\n  Example : Getting profile\n" << endl;

    long int prof[7];
    controller.getProfileData(prof[0],prof[1],prof[2],prof[3],prof[4],prof[5],prof[6]);

    // start profile velocity
    cout << "\n  Example : Profile Velocity (500 1/min) during 2 seconds\n" << endl;
    controller.setOperationMode(controller.PROFILE_VELOCITY);
    controller.setTargetProfileVelocity(500);
    controller.startProfileVelocity();
    sleep(2);
    controller.stopProfileVelocity();

    cout << "\n  Example : Profile Position (to 0)\n" << endl;
    controller.setTargetProfilePosition(0);
    controller.setOperationMode(controller.PROFILE_POSITION);
    controller.startProfilePosition(controller.ABSOLUTE,true,true);

    cout << "\n  Example : Closing\n" << endl;

    controller.close();

    cout << "\n  EPOS2 : GREEN LED blinking\n" << endl;

	}catch(std::exception &e){
		cout << e.what() << endl << endl;
	}

	return 1;

}

