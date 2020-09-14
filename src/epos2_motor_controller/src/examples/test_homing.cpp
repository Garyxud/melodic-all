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
 \example test_homing.cpp
 \brief Program for testing the Homing mode using a sensor
 \author Marti� Morta Garriga - IRI (CSIC-UPC)
 \date february 2011 (last revision)

 This program sets a digital input, in this case number 4 (default) (pin 5) as a
 home position and tests it,

 Change the parameters of the example is trivial.
 */

int main(void){

	CEpos2 controller;


	cout << "Epos2 Homing:" << endl;

	try{

    controller.init();
    controller.setVerbose(false);

    // enable motor in velocity mode
    controller.enableController();
    // Using sensor:
    //   11 Home Switch Negative Speed & Index
    //   7 Home Switch Positive Speed & Index
    //   27 Home Switch Negative Speed
    //   23 Home Switch Positive Speed
    controller.setHoming(11,600,50,1000,4);

    cout << "  DIGITAL INPUTS:\n";
    for(int i=1;i<=10;i++)
      cout << "   DigIN " << i << ": " << controller.getDigInState(i) << endl;
    cout
    << "\n   State:           " << bitset<16>(controller.getDigInStateMask())
    << "\n   Functionalities: " << bitset<16>(controller.getDigInFuncMask())
    << "\n   Polarity:        " << bitset<16>(controller.getDigInPolarity())
    << "\n   Execution:       " << bitset<16>(controller.getDigInExecutionMask())
    << "\n";

    controller.enableMotor(controller.HOMING);
    controller.doHoming(true);
    cout << "   Home is done!\n\n";

    controller.close();

	}catch(std::exception &e){
    controller.init();
    controller.close();
		cout << e.what() << endl;
	}

	return 1;

}

