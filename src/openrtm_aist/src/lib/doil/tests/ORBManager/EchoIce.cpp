#include <iostream>
#include <coil/Properties.h>
#include <doil/ice/IceManager.h>
#include "IEcho.h"
#include "EchoIceSkel.h"

//------------------------------------------------------------
// Echo servant for Ice
//------------------------------------------------------------
namespace EchoIce
{
class EchoServant
  : public virtual Demo::EchoSample,
    public virtual doil::Ice::IceServantBase
{
public:
  EchoServant(doil::ImplBase* impl)
    : doil::Ice::IceServantBase(impl)
  {
    std::cout << "EchoIceServant";
    m_impl = dynamic_cast<IEcho*>(impl);
    if (m_impl == NULL) throw std::bad_alloc();
    std::cout << " created." << std::endl;
  }
  virtual ~EchoServant()
  {
    std::cout << "EchoServant: " << name() << " deleted." << std::endl;
  }
  virtual void echo(const std::string& msg, const Ice::Current&)
  {
    std::cout << "servant" << name() << "::echo()  was called" << std::endl;
    m_impl->echo(msg);
  }
private:
  IEcho* m_impl;
};
};
extern "C"
{
  using namespace doil::Ice;
  void EchoIceInit(coil::Properties& prop)
  {
    std::cout << "EchoIceInit";
    IceManager::instance().registerFactory("EchoSample",
                                           doil::New<EchoIce::EchoServant>,
                                           doil::Delete<EchoIce::EchoServant>);
    std::cout << " done" << std::endl;
  }
}
