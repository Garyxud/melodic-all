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
#include <sstream>
#include "ctime.h"
#include "log.h"
#include "Epos2.h"

using namespace std;

/**
 * \example test_like_h3d.cpp
 *
 * This is a test to calculate average delays in epos2
 *
 */

int main(void){

	CEpos2 controller;
  CLog john("test_like_h3d");

	cout << "Epos2 Like H3D:" << endl;

  double avg=0,total_avg=0,max_avg=0,min_avg=0;
  int total_seconds=20;//3*60;

	try{

    controller.init();

    // enable motor in velocity mode
    controller.enableController();
    controller.enableMotor(controller.VELOCITY);
    controller.setOperationMode(controller.VELOCITY);
    //controller.setVerbose(true);

    controller.setTargetVelocity(350);

    // start the movement
    controller.startVelocity();
    CTime tbase,telapsed,t0,t1,tavg;
    int32_t pos=0;
    stringstream text;
    int k=0;

    telapsed.setFormat(ctf_ms);

    do{

      t0.set();
      pos = controller.readPosition();
      t1.set();
      tavg = t1-t0;
      avg = tavg.getTimeInSeconds();
      telapsed = (t1+t0)/2 - tbase;
      cout << ">>> " << telapsed.getTimeInSeconds() << " " << pos << endl;

      if(total_avg==0)
        total_avg = avg;
      else
        total_avg = (total_avg + avg)/2;

      max_avg = max(max_avg,avg);
      min_avg = min(min_avg,avg);

      telapsed.set();
      telapsed = telapsed - tbase;

      if( telapsed.getTimeInSeconds() > 5*k )
      {
        cout << telapsed << endl;
        k++;
      }
      //usleep(100*1000);

    }while( telapsed.getTimeInSeconds() < total_seconds);
    
    cout << "End" << endl;

    controller.stopVelocity();

    controller.close();

	}catch(CException &e){
    controller.close();
		cout << e.what() << endl;
	}

	cout <<
        " t avg: " << total_avg <<
        " t min: " << min_avg <<
        " t max: " << max_avg << endl;

	return 1;

}

