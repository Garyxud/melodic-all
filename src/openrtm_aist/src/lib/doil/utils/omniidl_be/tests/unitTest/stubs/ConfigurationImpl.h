// -*- C++ -*-
/*!
 * @file ConfigurationImpl.cpp 
 * @brief ConfigurationImpl C++ implementation sample for doil
 * @date $Date$
 * @author 
 *
 * $Id$
 */

#ifndef CONFIGURATION_IMPL_H
#define CONFIGURATION_IMPL_H

#include <doil/ImplBase.h>
#include <IConfiguration.h>
//#include <Logger.h>

namespace UnitTest
{
namespace Servant
{
  class Logger;
  /*!
   * @if jp
   * @class ConfigurationImpl
   * @brief ConfigurationServant試験用インプリメントクラス
   * @else
   * @class ConfigurationImpl
   * @brief Configuration implementation class for ConfigurationServant' unittest.
   * @endif
   */
  class ConfigurationImpl
   : public virtual doil::ImplBase,
     public virtual SDOPackage::Local::IConfiguration
  {
  public:
    ConfigurationImpl();
    ConfigurationImpl(Logger& aLogger);

    virtual ~ConfigurationImpl();

    virtual bool set_device_profile(const ::SDOPackage::Local::DeviceProfile& dProfile)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool set_service_profile(const ::SDOPackage::Local::ServiceProfile& sProfile)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool add_organization(const ::SDOPackage::Local::IOrganization* org)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool remove_service_profile(const ::std::string& id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool remove_organization(const ::std::string& organization_id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual ::SDOPackage::Local::ParameterList get_configuration_parameters()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual ::SDOPackage::Local::NVList get_configuration_parameter_values()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual ::std::string get_configuration_parameter_value(const ::std::string& name)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool set_configuration_parameter(const ::std::string&name, const ::std::string& value)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual ::SDOPackage::Local::ConfigurationSetList get_configuration_sets()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual ::SDOPackage::Local::ConfigurationSet get_configuration_set(const ::std::string& config_id)
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool set_configuration_set_values(const ::std::string&config_id, const ::SDOPackage::Local::ConfigurationSet& configuration_set)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual ::SDOPackage::Local::ConfigurationSet get_active_configuration_set()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool add_configuration_set(const ::SDOPackage::Local::ConfigurationSet& configuration_set)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool remove_configuration_set(const ::std::string& config_id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    virtual bool activate_configuration_set(const ::std::string& config_id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError);

    const char* id() { return "Configuration"; }
    const char* name() { return m_name; }
    void incRef() { refcount++; }
    void decRef() { refcount--; }
  private:
    static int count;
    char m_name[32];
    int refcount;
    Logger *m_logger;
  };
}; // namespace Local 
}; // namespace SDOPackage 

#endif  // CONFIGURATION_IMPL_H
// End of File.
