#include <doil/ImplBase.h>
#include <string>
#include <iostream>
#include "IEcho.h"
//------------------------------------------------------------
// Implementation class
//------------------------------------------------------------
class EchoImpl
  : public IEcho,
    public doil::ImplBase
{
public:
  EchoImpl();
  virtual ~EchoImpl();
  const char* id();
  const char* name();
  void incRef();
  void decRef();
  void echo(std::string msg);
  static int count;
  char m_name[16];
};

