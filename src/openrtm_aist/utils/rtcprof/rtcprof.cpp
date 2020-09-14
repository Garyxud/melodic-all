// -*- C++ -*-
/*!
 * @file rtcprof.cpp
 * @brief RT-Component profile dump command
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2010
 *     Noriaki Ando
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <iostream>
#include <string>
#include <vector>

#include <coil/Properties.h>
#include <coil/File.h>

#include <rtm/Manager.h>


int main(int argc, char* argv[])
{
  if (argc != 2)
    {
      std::cerr << "usage: " << std::endl;
      std::cerr << argv[0] << " .so or .DLL" << std::endl;
      std::cerr << std::endl;
      return -1;
    }

  //
  // notice: coil::dirname brakes original string
  // 
  // file name with full path
  std::string fullname(argv[1]);
  // directory name
  std::string dirname(coil::dirname(argv[1]));
  // basename
  std::string basename(coil::basename(fullname.c_str()));

  // making command line option
  //   dummy -o manager.modules.load_path
  coil::vstring opts;
  opts.push_back("dummy");
  opts.push_back("-o");
  std::string load_path("manager.modules.load_path:");
  load_path += dirname;
  opts.push_back(load_path);
  opts.push_back("-o");
  opts.push_back("logger.enable:NO");
  opts.push_back("-o");
  opts.push_back("manager.corba_servant:NO");

  // Manager initialization
  RTC::Manager::init(opts.size(), coil::toArgv(opts));
  RTC::Manager& mgr(RTC::Manager::instance());


  // loaded profile = old profiles - new profiles
  std::vector<coil::Properties> oldp(mgr.getFactoryProfiles());
  mgr.load(basename.c_str(), "");
  std::vector<coil::Properties> newp(mgr.getFactoryProfiles());
  std::vector<coil::Properties> profs;

  for (size_t i(0); i < newp.size(); ++i)
    {
      bool exists(false);
      for (size_t j(0); j < oldp.size(); ++j)
        {
          if (oldp[j]["implementation_id"] == newp[i]["implementation_id"] &&
              oldp[j]["type_name"]         == newp[i]["type_name"] &&
              oldp[j]["description"]       == newp[i]["description"] &&
              oldp[j]["version"]           == newp[i]["version"])
            {
              exists = true;
            }
        }
      if (!exists) { profs.push_back(newp[i]); }
    }

  // loaded component profile have to be one
  if (profs.size() == 0)
    {
      std::cerr << "Load failed." << std::endl;
      return -1;
    }

  if (profs.size() > 1)
    {
      std::cerr << "One ore more modules loaded." << std::endl;
      return -1;
    }

  coil::vstring keys(profs[0].propertyNames());

  for (size_t i(0); i < keys.size(); ++i)
    {
      std::cout << keys[i] << ": " << profs[0][keys[i]] << std::endl;
    }

  return 0;
}
