// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Martí Morta Garriga  (mmorta@iri.upc.edu)
// All rights reserved.
//
// This file is part of IRI EPOS2 Driver
// IRI EPOS2 Driver is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>
#include "Epos2.h"

using namespace std;

/*!
 \example test_marker.cpp
 \brief Program for testing Position Marker Sensor
 \author Marti� Morta Garriga - IRI (CSIC-UPC)
 \date february 2011 (last revision)

 This program sets a digital input, in this case number 2 (pin 7) as a position
 marker and tests it, leaving the motor free as you can move your load and see
 how the marker catches positions until 10 times.

 Change the parameters of the example or make it automatic using Velocity mode
 is trivial.
 */

int main(void){

	CEpos2 controller;


	cout << "Epos2 Position Marker:" << endl;

	try{

    controller.init();
    controller.setVerbose(false);

    // enable motor in velocity mode
    controller.enableController();

    char digin = 2;
    controller.setPositionMarker(0,0,0,digin);

    cout << "  DIGITAL INPUTS:\n";
    for(int i=1;i<=10;i++)
      cout << "   DigIN " << i << ": " << controller.getDigInState(i) << endl;
    cout
    << "\n   State:           " << bitset<16>(controller.getDigInStateMask())
    << "\n   Functionalities: " << bitset<16>(controller.getDigInFuncMask())
    << "\n   Polarity:        " << bitset<16>(controller.getDigInPolarity())
    << "\n   Execution:       " << bitset<16>(controller.getDigInExecutionMask())
    << "\n";

   // if(controller.getDigInState(digin) == 4 && false)
   // {
     // controller.enableMotor(controller.VELOCITY);
     // controller.setTargetVelocity(300);
     // controller.startVelocity();

      int num_positions = 1;

      while(num_positions < 11){
        controller.waitPositionMarker();
        cout << "  > New Position: " << endl
             << "      Hist(0): " << controller.getPositionMarker()  << endl
             << "      Hist(1): " << controller.getPositionMarker(1) << endl
             << "      Hist(2): " << controller.getPositionMarker(2) << endl;
        num_positions++;
      }
     // controller.stopVelocity();
    // }
    controller.close();

	}catch(std::exception &e){
    controller.init();
    controller.close();
		cout << e.what() << endl;
	}

	return 1;

}

