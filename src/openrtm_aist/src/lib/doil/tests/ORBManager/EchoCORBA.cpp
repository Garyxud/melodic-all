#include <iostream>
#include <coil/Properties.h>
#include <doil/corba/CORBAManager.h>
#include "IEcho.h"
#include "EchoCORBASkel.h"

//------------------------------------------------------------
// Echo servant for CORBA
//------------------------------------------------------------
class EchoServant
  : public virtual doil::CORBA::CORBAServantBase,
    public virtual POA_EchoCORBA::EchoSample
{
public:
  EchoServant(doil::ImplBase* impl)
    : doil::CORBA::CORBAServantBase(impl)
  {
    std::cout << "EchoCORBAServant" << std::endl;
    std::cout << "id  : " << impl->id() << std::endl;
    std::cout << "name: " << impl->name() << std::endl;
    m_impl = dynamic_cast<IEcho*>(impl);
    //    std::cout << "id  : " << m_impl->id() << std::endl;
    //    std::cout << "name: " << m_impl->name() << std::endl;
    if (m_impl == NULL) 
      {
        std::cout << "Error!!!!!: cast failed in EchoCORBAServant" << std::endl;
        throw std::bad_alloc();
      }
    std::cout << " was created" << std::endl;
  }
  virtual ~EchoServant()
  {
    std::cout << "EchoServant: " << name() << " deleted." << std::endl;
  }
  void echo(const char* msg)
  {
    m_impl->echo(msg);
  }
private:
  IEcho* m_impl;
};

extern "C"
{
  using namespace doil::CORBA;
  void EchoCORBAInit(coil::Properties& prop)
  {
    std::cout << "EchoCORBAInit";
    CORBAManager::instance().registerFactory("EchoSample",
                                             doil::New<EchoServant>,
                                             doil::Delete<EchoServant>);
    std::cout << " done" << std::endl;
  }
}
