/*
** Author(s):
**  - Cedric GESTES <gestes@aldebaran.com>
**
** Copyright (C) 2014 Aldebaran
*/
#include "logproviderimpl.hpp"
#include <qi/anymodule.hpp>

namespace qi
{
  void _qiregisterProgressNotifier();
  void _qiregisterProgressNotifierProxy();
  void _qiregisterFile();
  void _qiregisterFileProxy();
  void _qiregisterFileOperation();

  void registerProgressNotifierCreation(qi::ModuleBuilder& mb);
  void registerFileCreation(qi::ModuleBuilder& mb);
  void registerFileOperations(qi::ModuleBuilder& mb);
}

static bool _qiregisterFileTypes()
{
  qi::_qiregisterProgressNotifier();
  qi::_qiregisterProgressNotifierProxy();
  qi::_qiregisterFile();
  qi::_qiregisterFileProxy();
  qi::_qiregisterFileOperation();
  return true;
}

static bool __qi_registrationFileTypes = _qiregisterFileTypes();

void registerLibQiCore(qi::ModuleBuilder* mb)
{
  qi::registerProgressNotifierCreation(*mb);
  qi::registerFileCreation(*mb);
  qi::registerFileOperations(*mb);
  qi::registerLogProvider(mb);
}

QI_REGISTER_MODULE("qicore", &registerLibQiCore);
