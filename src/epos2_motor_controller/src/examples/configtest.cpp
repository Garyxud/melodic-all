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

/**
 * \example configtest.cpp
 * \brief Program for configure and test EPOS2
 * \author Martí Morta Garriga - IRI (CSIC-UPC)
 * \date march 2010 (last revision)
 *
 * This is a program to configure and test an EPOS2 Controller unit,
 *
 */



/**
			INCLUDES
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <cstdio>
#include <cstring>
#include <ctime>

#include <iostream>
#include <fstream>
#include <vector>

#include <time.h>
#include <sys/time.h>

#include "Epos2.h"


/**
			GLOBALS
*/
bool finish_program=false;
bool free_program=true;
CEpos2 *epos2;
bool blocking=false,print=false,waiting=true;


using namespace std;


/*!
	\brief motor [qc] to platform [°]

	\param qc angle from readPosition Motor [qc]
	\return platform angle [°]

*/
float mqc2pdg(long int qc)
{
	return((float)qc*3/3700);
}

/*!
	\brief platform [°] to motor [qc]
       It truncates the value

	\param dg platform angle [°]
	\return motor angle [qc]

*/
long int pdg2mqc(float dg)
{
	return((long int)(dg*3700/3));
}


/** Read Data
 * \brief reads all epos2 data
 *
 * */
void readData(){

	long int li;
	int i;
	li=epos2->readVelocity();
	cout << "    Velocity: " << hex << "0x" << li << " , " << dec << li << endl;
	li=epos2->readVelocitySensorActual();
	cout << "    Velocity Sensor Actual: " << hex << "0x" << li << " , " << dec << li << endl;
	li=epos2->readVelocityDemand();
	cout << "    Velocity Demand: " << hex << "0x" << li << " , " << dec << li << endl;
	li=epos2->readVelocityActual();
  cout << "    Velocity Actual: " << hex << "0x" << li << " , " << dec << li << endl;
	li=epos2->readPosition();
  cout << "    Position: " << hex << "0x" << li << " , " << dec << li << endl;
	li=epos2->readEncoderCounter();
  cout << "    Encoder Counter: " << hex << "0x" << li << " , " << dec << li << endl;
	li=epos2->readEncoderCounterAtIndexPulse();
  cout << "    Encoder Counter @ index pulse: " << hex << "0x" << li << " , " << dec << li << endl;
	i=epos2->readCurrent();
  cout << "    Current: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readCurrentAveraged();
  cout << "    Current Averaged: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readCurrentDemanded();
  cout << "    Current Demanded: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readStatusWord();
  cout << "    StatusWord: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readHallsensorPattern();
  cout << "    Hall Sensor Pattern: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readFollowingError();
  cout << "    Following Error: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readVersionHardware();
  cout << "    Version Hardware: " << hex << "0x" << i << " , " << dec << i << endl;
	i=epos2->readVersionSoftware();
  cout << "    Version Software: " << hex << "0x" << i << " , " << dec << i << endl;
}





/** doMovement
 * \brief Starts a motor movement
 *
 * \param type a:Profile Position Absolute
 * r: Profile Position Relative
 * w: Profile Velocity
 * v: Velocity
 * */
void doMovement(char type){

	long int value;
	char ui_r;
  CEpos2::epos_posmodes mode;
  bool opcio_target=true;

	epos2->enableOperation();
	switch(type){
		case 'a':
      cout << "  ConfigTest  Absolute Movement\n\n";
		case 'r':
      cout << "  ConfigTest  Relative Movement\n\n";
			value=epos2->getTargetProfilePosition();
			break;
		case 'w':
      cout << "  ConfigTest  Profle Velocity\n\n";
			value=epos2->getTargetProfileVelocity();
			break;
		case 'v':
      cout << "  ConfigTest  Velocity\n\n";
			value=epos2->getTargetVelocity();
			break;
    case 'k':
      opcio_target = false;
      break;
	}
  if(opcio_target)
  {
    cout << "    Is this your desired target? (Y,n)  " << value << endl;
    cin >> ui_r;
    cout << endl;
	  if(ui_r=='n'){
      cout << "	  Type your desired target  " << endl;
		  cin >> value;
	  }
  }
	switch(type){
		case 'a':
		case 'r':
			epos2->setOperationMode(epos2->PROFILE_POSITION);
			epos2->setTargetProfilePosition(value);
			value = epos2->getTargetProfilePosition();
			cout << "    Starting Profile Position to:" << value;
      if(type == 'a') mode = epos2->ABSOLUTE;
      if(type == 'r') mode = epos2->RELATIVE;
			epos2->startProfilePosition(mode,blocking,waiting); // no blocking

			if(!blocking){
				while(!epos2->isTargetReached()){
					epos2->getMovementInfo();
				}
			}

			cout << "\n    Profile Position Movement Finished" << endl;
			break;
		case 'w':
			int wcounter;
      epos2->setOperationMode(epos2->PROFILE_VELOCITY);
			epos2->setTargetProfileVelocity(value);
			value=epos2->getTargetProfileVelocity();
			cout << "    Starting Profile Velocity to:" << value;
			cout << "     during 5 seconds" << endl;
			epos2->startProfileVelocity();
			wcounter=0;
			cout << "    TARGET VELOCITY REACHED - Press 's' to stop'" << endl;
			while(wcounter<100){
				epos2->getMovementInfo();
				wcounter++;
			}
			epos2->stopProfileVelocity();
			cout << "\n    Profile Position Movement Finished" << endl;
			break;
		case 'v':
			char ui_v_stop;
      epos2->setOperationMode(epos2->VELOCITY);
			epos2->setTargetVelocity((int)value);
			epos2->startVelocity();
			cout << "    VELOCITY - Press 's' to stop'" << endl;
      cin >> ui_v_stop;
			while(ui_v_stop!='s'){
				epos2->getMovementInfo();
				cin >> ui_v_stop;
			}
			epos2->stopVelocity();
			cout << "    Velocity Movement Finished" << endl;
			break;
    case 'k':
      /*
      epos2->setProfileData(200,2000,2000,4000,2000,4500,1);
      cout << "  1 velocity + profile velocity (400)" << endl;
      epos2->setOperationMode(epos2->VELOCITY);
      epos2->setTargetVelocity(400);
      cout << "      velocity" << endl;
      epos2->startVelocity();
      sleep(2);
      epos2->stopVelocity();
      cout << "      stop velocity" << endl;
      epos2->setOperationMode(epos2->PROFILE_VELOCITY);
      epos2->setTargetProfileVelocity(1000);
      cout << "      profile velocity" << endl;
      epos2->startProfileVelocity();
      sleep(4);
      epos2->stopProfileVelocity();
      cout << "      stop profile velocity" << endl;
      sleep(2);
      cout << "  2 profile velocity with vel change" << endl;
      epos2->setOperationMode(epos2->PROFILE_VELOCITY);
      epos2->setTargetProfileVelocity(200);
      cout << "      1st" << endl;
      epos2->startProfileVelocity();
      sleep(4);
      epos2->stopProfileVelocity();
      cout << "      2nd" << endl;
      epos2->stopProfileVelocity();
      sleep(1);
      epos2->setTargetProfileVelocity(1800);
      epos2->startProfileVelocity();
      sleep(2);
      epos2->stopProfileVelocity();
      sleep(2);
      cout << "  3 velocity with vel change (400 -> 800)" << endl;
      epos2->setOperationMode(epos2->VELOCITY);
      cout << "      600" << endl;
      epos2->setTargetVelocity(600);
      epos2->startVelocity();
      sleep(5);
      cout << "      1000" << endl;
      epos2->setTargetVelocity(1000);
      sleep(5);
      epos2->stopVelocity();
      */
      cout << "  4 profile position with change wait without blocking" << endl;
      epos2->setOperationMode(epos2->PROFILE_POSITION);
      epos2->setProfileData(500,2000,2000,4000,2000,4500,1);
      epos2->setTargetProfilePosition(222000);
      cout << "      Start Relative not blocking not waiting" << endl;
      epos2->startProfilePosition(epos2->RELATIVE, false, false);
      sleep(2);
      //cout << "      Halt (blocking)" << endl;
      //epos2->startProfilePosition(epos2->HALT, false, false);
      epos2->setTargetProfilePosition(50000);
      epos2->setProfileData(3000,3000,3000,4000,3500,4500,1);
      cout << "      Relative blocking waiting" << endl;
      epos2->startProfilePosition(epos2->RELATIVE, true, false);
      epos2->setTargetProfilePosition(-20000);
      epos2->startProfilePosition(epos2->RELATIVE, true, false);
      epos2->setTargetProfilePosition(50000);
      epos2->startProfilePosition(epos2->RELATIVE, true, false);
      //cout << "  5 profile position with change not waiting" << endl;


      cout << "    End Test" << endl;
	}
}

/** throwAction
 * \brief throw an action set by user
 *
 * \param action A char which defines what action user wants
 * */

void throwAction(char action)
{

  char cs; // character select

	switch( action ){
		case 'a':
		case 'r':
		case 'w':
		case 'v':
    case 'k':
			doMovement(action);
			break;
		case 'h':
      cout << "  ConfigTest  Set Home\n\n";
			epos2->setHome();
			break;
   break;
		case '1':
      // Configure
      cout << "  ConfigTest  Configure Profile Parameters\n\n";
			// Profile

      long prof[7];

epos2->getProfileData(prof[0],prof[1],prof[2],prof[3],prof[4],prof[5],prof[6]);

      cout
          << "    PROFILE DATA"                              << endl
          << "   "
 << dec << endl
          << "   Velocity:   "      << prof[0] << "[rpm]"    << endl
          << "   Max Velocity:   "  << prof[1] << "[rpm]"    << endl
          << "   Acceleration:   "  << prof[2] << "[rpm/s]"  << endl
          << "   Deceleration:   "  << prof[3] << "[rpm/s]"  << endl
          << "   QS Decel: "        << prof[4] << "[rpm/s]"  << endl
          << "   Max Accel:   "     << prof[5] << "[rpm/s]"  << endl
          << "   Type:     "        << prof[6]               << endl<<endl;

      cout << "    Profile: Do you want to configure? (y,n): " ;
      cin >> cs;
      while(cs!='y' && cs!='n')
      {
        cout << endl << "    Input Error. press 'y' or 'n': ";
        cin >> cs;
      }
      cout << endl;
      if(cs=='y'){

        cout << " Velocity [rpm] "       << endl;
        cin >> prof[0]; cout << endl;
        cout << " Max Velocity [rpm] "   << endl;
        cin >> prof[1]; cout << endl;
        cout << " Acceleration [rpm/s] " << endl;
        cin >> prof[2]; cout << endl;
        cout << " Deceleration [rpm/s] " << endl;
        cin >> prof[3]; cout << endl;
        cout << " QS Decel [rpm/s] "     << endl;
        cin >> prof[4]; cout << endl;
        cout << " Max acc [rpm/s] "     << endl;
        cin >> prof[5]; cout << endl;
        cout << " Type "                 << endl;
        cin >> prof[6]; cout << endl;


epos2->setProfileData(prof[0],prof[1],prof[2],prof[3],prof[4],prof[5],prof[6]);

      }
			break;
		case '2':
      cout << "  ConfigTest  Configure Control Parameters\n\n";
			// Control
			long conpar[10];

epos2->getControlParameters(conpar[0],conpar[1],conpar[2],conpar[3],conpar[4],
conpar[5],conpar[6],conpar[7],conpar[8],conpar[9]);

      cout
          << "    CONTROL PARAMETERS DATA"       << endl
   << dec << endl
          << "   Current P:   "     << conpar[0] << endl
          << "   Current I:   "     << conpar[1] << endl
          << "   Velocity P:   "    << conpar[2] << endl
          << "   Velocity I:   "    << conpar[3] << endl
          << "   Velocity SPF:   "  << conpar[4] << endl
          << "   Position P:   "    << conpar[5] << endl
          << "   Position I:   "    << conpar[6] << endl
          << "   Position D:   "    << conpar[7] << endl
          << "   Position Vff:   "  << conpar[8] << endl
          << "   Position Aff:   "  << conpar[9] << endl
          << endl;

      cout << "    Profile: Do you want to configure? (y,n): " ;
      cin >> cs;
      while(cs!='y' && cs!='n')
      {
        cout << endl << "    Input Error. press 'y' or 'n': ";
        cin >> cs;
      }
      cout << endl;
      if(cs=='y'){

        cout << "   Current P: "       << endl;
        cin >> conpar[0]; cout << endl;
        cout << "   Current I: "     << endl;
        cin >> conpar[1]; cout << endl;
        cout << "   Velocity P: "    << endl;
        cin >> conpar[2]; cout << endl;
        cout << "   Velocity I: "    << endl;
        cin >> conpar[3]; cout << endl;
        cout << "   Velocity SPF: "  << endl;
        cin >> conpar[4]; cout << endl;
        cout << "   Position P: "    << endl;
        cin >> conpar[5]; cout << endl;
        cout << "   Position I: "    << endl;
        cin >> conpar[6]; cout << endl;
        cout << "   Position D: "    << endl;
        cin >> conpar[7]; cout << endl;
        cout << "   Position Vff: "  << endl;
        cin >> conpar[8]; cout << endl;
        cout << "   Position Aff: "  << endl;
        cin >> conpar[9]; cout << endl;


epos2->setControlParameters(conpar[0],conpar[1],conpar[2],conpar[3],conpar[4],
conpar[5],conpar[6],conpar[7],conpar[8],conpar[9]);

      }

			break;
		case '3':
			// Sensor
      cout << "  ConfigTest  Configure Sensor Parameters\n\n";
			break;
		case '4':
			// Motor
      cout << "  ConfigTest  Configure Motor Parameters\n\n";
			break;
		case '5':
			// Position
      cout << "  ConfigTest  Configure Position Parameters\n\n";
			break;
		case '6':
			// Units Dimension
      cout << "  ConfigTest  Configure Unit Dimensions\n\n";
			break;
		case '7':
			// Communication
      cout << "  ConfigTest  Configure Communication Parameters\n\n";
			break;
		case '8':
			// Save all parameters
      cout << "  ConfigTest  Saving Parameters\n\n";
      cout << "    Are you sure you want to save parameters? (Y,n)  " << endl;
			char sure8;
			cin >> sure8;
			if(sure8=='Y'){
				epos2->saveParameters();
				cout << "    Parameters Saved" << endl;
			}else{
				cout << "    parameters not saved" << endl;
			}
			break;
		case '9':
			// Restore Default Parameters
      cout << "  ConfigTest  Restore Default Parameters\n\n";
      cout << "    Are you sure you want to restore parameters? (Y,n)  " <<
endl;
			char sure9;
			cin >> sure9;
			if(sure9=='Y'){
			 epos2->restoreDefaultParameters();
			 cout << "    Parameters Restored" << endl;
			}else{
			 cout << "    parameters not restored" << endl;
      }
			break;
		case 'E':
			// Read
      cout << "  ConfigTest  Read Errors\n\n";
			// Error
			long *errors;
			epos2->readError();
			epos2->readErrorHistory(&errors);
			break;
		case 'S':
			// StatusWord
      cout << "  ConfigTest  Read Status Word\n\n";
			cout << "    STATUS-WORD: " << hex << epos2->readStatusWord() << endl;
			break;
		case 'D':
			// Data
      cout << "  ConfigTest  Read Data\n\n";
			readData();
			break;
		case 'x':
		case 'X':
			// Exit
			finish_program=true;
			break;
    default:
      cout << "  Choose another option" << endl << endl;
      break;
	}

}



/**
\b MAIN
\brief Programa de prova per a obtenir escaneigs i posicions del motor

**/

int main(int argc, char **argv)
{
	//  Variable declaration
	char action;
	bool verbose = false;
	int opt;

  cout << "\n  EPOS2 CONFIG&TEST \n\n";


	// Get some options from the command line
	while ((opt = getopt(argc, argv, "biphv")) != -1)
	{
		switch (opt)
		{
			case 'b':
				// blocking
				blocking = true;
				break;
      case 'i':
        waiting = false;
        break;
			case 'p':
				// print
				print = true;
				break;
      case 'v':
        verbose=true;
        break;
			case '?':
			case 'h':
			default:
        cout << "  USAGE" << endl << "\n"
            << "  " << argv[0] << " [options]" << endl << endl
            << "  OPTIONS" << endl <<
            "\n"
            << "  -i Interrupt Movements" <<
            "\tin profile position.\n" << endl
            << "  -v Verbose" <<
            "\tShows all information\n" << endl << endl;
				return 1;
		}
	}
	epos2 = new CEpos2();

  cout << "  MENU" << endl <<
      "\n"
      << "  ▾ Do a movement" << endl
      << "  a" <<
      "\tProfile Position Absolute\n"
      << "  r" <<
      "\tProfile Position Relative\n"
      << "  w" <<
      "\tProfile Velocity\n"
      << "  v" <<
      "\tVelocity\n"
      << "  k" <<
      "\tKombo\n"
      << "  h" <<
      "\tSet Home\n" << endl
      << "  Configure Parameters" << endl
      << "  1" <<
      "\tProfile\n"
      << "  2" <<
      "\tControl\n"
      << "  3" <<
      "\tSensor\n"
      << "  4" <<
      "\tMotor (not done)\n"
      << "  5" <<
      "\tPosition (not done)\n"
      << "  6" <<
      "\tUnits dimension (not done)\n"
      << "  7" <<
      "\tCommunication (not done)\n"
      << "  8" <<
      "\tSave All Parameters\n"
      << "  9" <<
      "\tRestore Default\n"  << endl
      << "  Read" << endl
      << "  E" <<
      "\tError\n"
      << "  S" <<
      "\tStatusword\n"
      << "  D" <<
      "\tData\n"
      << endl
      << "  x" <<
      "\tExit Program\n"
      << endl << endl;

  // Start MOTOR Controller
  try
  {
    cout << "  ConfigTest init EPOS2\n\n";

    epos2->init();

    epos2->setVerbose(verbose);


    // Check Error
    long *errors;
    epos2->readError();
    epos2->readErrorHistory(&errors);

    // enable motor
    cout << "  ConfigTest  enable controller\n\n";
    epos2->enableController();

    cout << "  ConfigTest  enable motor\n\n";
    epos2->enableMotor(epos2->PROFILE_POSITION);

    cout << "  ConfigTest  disable operation\n\n";
    epos2->disableOperation();
    // Do the program while not finish it

    while(!finish_program){
      cout << "  ? ";
      cin >> action;
      // Wait for action
      cout << endl;
      throwAction(action);
    }

    // Stop motor controller
    epos2->close();

  }catch(std::exception &exc)
  {
    cout << "EPOS2 Exception: " << exc.what() << endl;
    return(-1);
  }
  //  Program End
  return(0);
}


