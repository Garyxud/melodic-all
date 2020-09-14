#include <doil/ImplBase.h>
#include "EchoImpl.h"

//------------------------------------------------------------
// Implementation class
//------------------------------------------------------------
EchoImpl::EchoImpl()
{
  sprintf(m_name, "EchoSample%d", count);
  ++count;
}
EchoImpl::~EchoImpl()
{
  std::cout << "EchoImpl: " << name() << " deleted." << std::endl;
}
const char* EchoImpl::id() {return "EchoSample";}
const char* EchoImpl::name() {return m_name;}
void EchoImpl::incRef(){}
void EchoImpl::decRef(){}
void EchoImpl::echo(std::string msg)
{
  std::cout << name() <<  " -> Message is: " << msg << std::endl;
  return;
}
int EchoImpl::count = 0;


