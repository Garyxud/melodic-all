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
#include "ctime.h"
#include "Epos2.h"

using namespace std;

/**
 * \example test_positions.cpp
 *
 * This is a test to calculate if the position acquisition is correct
 *
 */

int main(void){

	CEpos2 controller;

	cout << "Epos2 Positions:" << endl;

  long pos_marker=0,pos_marker_old=0;
	CTime tavg;

	try{

    controller.init();

    // enable motor in velocity mode
    controller.enableController();
    controller.enableMotor(controller.VELOCITY);
    controller.setOperationMode(controller.VELOCITY);

    CTime tbase,telapsed;
    int experiment_duration = 10, vel = 1000;
    bool primera = true;

    controller.setPositionMarker();
    controller.setTargetVelocity(vel);
    controller.startVelocity();


    cout << "t0\tpos0\tt1\tpos1"<< endl;

    do{

      telapsed.set();
      telapsed = telapsed - tbase;

      pos_marker = controller.getPositionMarker(1);


      if (pos_marker != pos_marker_old)
      {
        telapsed.setFormat(ctf_ms);
        if(primera){
          cout << telapsed << "\t" << pos_marker;
          primera = false;
        }else{
          cout << "\t" << telapsed << "\t" << pos_marker << endl;
          primera = true;
        }
        pos_marker_old = pos_marker;
      }

    }while( telapsed.getTimeInSeconds() < experiment_duration );

    controller.stopVelocity();

    controller.close();

	}catch(CException &e){
    controller.close();
		cout << e.what() << endl;
	}

	return 1;

}

