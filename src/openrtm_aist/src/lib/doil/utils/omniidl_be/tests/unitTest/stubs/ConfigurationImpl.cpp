// -*- C++ -*-
/*!
 * @file ConfigurationImpl.cpp 
 * @brief ConfigurationImpl C++ implementation sample for doil
 * @date $Date$
 * @author 
 *
 * $Id$
 */


#include <ConfigurationImpl.h>
#include <iostream>
#include <Logger.h>

namespace UnitTest
{
namespace Servant
{
  /*!
   * @if jp
   * @class ConfigurationImpl
   * @brief ConfigurationServant試験用インプリメントクラス
   * @else
   * @class ConfigurationImpl
   * @brief Configuration implementation class for ConfigurationServant' unittest.
   * @endif
   */
   int ConfigurationImpl::count = 0;

    ConfigurationImpl::ConfigurationImpl()
    {
      sprintf(m_name, "%s%d", id(), count);
      ++count;
      m_logger = new Logger();
//      std::cout << "ConfigrationImpl: " << name() << " created." << std::endl;
    }

    ConfigurationImpl::ConfigurationImpl(Logger& aLogger)
    {
      sprintf(m_name, "%s%d", id(), count);
      ++count;
      m_logger = &aLogger;
//      std::cout << "ConfigrationImpl: " << name() << " created." << std::endl;
    }

     ConfigurationImpl::~ConfigurationImpl()
    {
//      std::cout << "ConfigrationImpl: " << name() << " deleted." << std::endl;
    };

     bool ConfigurationImpl::set_device_profile(const ::SDOPackage::Local::DeviceProfile& dProfile)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("set_device_profile");
      return true;
    }

     bool ConfigurationImpl::set_service_profile(const ::SDOPackage::Local::ServiceProfile& sProfile)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("set_service_profile");
      return true;
    }

     bool ConfigurationImpl::add_organization(const ::SDOPackage::Local::IOrganization* org)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("add_organization");
      return true;
    }

     bool ConfigurationImpl::remove_service_profile(const ::std::string& id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("remove_service_profile");
      m_logger->push(id.c_str());
      return true;
    }

     bool ConfigurationImpl::remove_organization(const ::std::string& organization_id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("remove_organization");
      m_logger->push(organization_id.c_str());
      return true;
    }

     ::SDOPackage::Local::ParameterList ConfigurationImpl::get_configuration_parameters()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("get_configuration_parameters");
      ::SDOPackage::Local::ParameterList result;
      return result;
    }

     ::SDOPackage::Local::NVList ConfigurationImpl::get_configuration_parameter_values()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("get_configuration_parameter_values");
      ::SDOPackage::Local::NVList result;
      return result;
    }

     ::std::string ConfigurationImpl::get_configuration_parameter_value(const ::std::string& name)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("get_configuration_parameter_value");
      m_logger->push(name.c_str());
      ::std::string result(name);
      return result;
    }

     bool ConfigurationImpl::set_configuration_parameter(const ::std::string&name, const ::std::string& value)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("set_configuration_parameter");
      m_logger->push(name.c_str());
      return true;
    }

     ::SDOPackage::Local::ConfigurationSetList ConfigurationImpl::get_configuration_sets()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("get_configuration_sets");
      ::SDOPackage::Local::ConfigurationSetList result;
      return result;
    }

     ::SDOPackage::Local::ConfigurationSet ConfigurationImpl::get_configuration_set(const ::std::string& config_id)
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("get_configuration_set");
      ::SDOPackage::Local::ConfigurationSet result;
      m_logger->push(config_id.c_str());
      return result;
    }

     bool ConfigurationImpl::set_configuration_set_values(const ::std::string&config_id, const ::SDOPackage::Local::ConfigurationSet& configuration_set)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("set_configuration_set_values");
      m_logger->push(config_id.c_str());
      return true;
    }

     ::SDOPackage::Local::ConfigurationSet ConfigurationImpl::get_active_configuration_set()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("get_active_configuration_set");
      ::SDOPackage::Local::ConfigurationSet result;
      return result;
      
    }

     bool ConfigurationImpl::add_configuration_set(const ::SDOPackage::Local::ConfigurationSet& configuration_set)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("add_configuration_set");
      return true;
    }

     bool ConfigurationImpl::remove_configuration_set(const ::std::string& config_id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("remove_configuration_set");
      m_logger->push(config_id.c_str());
      return true;
    }

     bool ConfigurationImpl::activate_configuration_set(const ::std::string& config_id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
      m_logger->push("activate_configuration_set");
      m_logger->push(config_id.c_str());
      return true;
    }

}; // namespace Local 
}; // namespace SDOPackage 

// End of File.
