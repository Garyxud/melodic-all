// -*- C++ -*-
/*!
 * @file rtcsh.cpp
 * @brief RTComponent manager shell
 * @date $Date: 2005-05-12 09:06:19 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2005
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <iostream>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>
#include <vector>
#include <string>
#include <rtm/RtcNaming.h>
#include <rtm/RtcConfig.h>
#include <rtm/RtcManager.h>

using namespace RTM;
using namespace std;


int main(int argc, char** argv)
{
  RtcConfig config(argc, argv);
  RtcNaming naming;
  CORBA::ORB_var orb;

  char** _argv = config.getOrbInitArgv();
  int    _argc = config.getOrbInitArgc();

  try {
	orb = CORBA::ORB_init(_argc, _argv);
	cout << "ORB init done" << endl;
  }
  catch (...) {
	cout << "error cought" << endl;
  }
  
  naming.initNaming(orb);
  cout << "Naming init done" << endl;

  string mgrname(".*/.*/Manager.*");
  vector<CORBA::Object_ptr> vObj;
  naming.findManager(mgrname, vObj);

  cout << "Number of objects:" << vObj.size() << endl;

  if (vObj.size() == 0) {
	cout << "Manager could not find." << endl;
	return -1;
  }

  CORBA::Object_ptr obj = *(vObj.end()-1);
  if (CORBA::is_nil(obj)) {
	cout << "manager not found" << endl;
	return 0;
  }

  RTM::RTCManager_var mgr = RTM::RTCManager::_narrow(obj);
  if (CORBA::is_nil(mgr)) {
	cout << "manager not found" << endl;
	return 0;
  }
 
  char* retval;

  string cmd;
  while (1) {
	cout << ">> ";
	getline(cin, cmd);

	//	retval = NULL;
	if (cmd.size() > 0 && !CORBA::is_nil(mgr)) {
	  bool ret = mgr->command(cmd.c_str(), retval);

	  if (ret == true) {
		//		cout << "Success: " <<  endl;
		cout << "Success: " << retval <<  endl;
		//		CORBA::string_free(retval);
	  } else {
		cout << "Error" << endl;
	  }
	}
  }
  return 0;
}
