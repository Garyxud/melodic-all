// -*- C++ -*-
/*!
 * @file ConnectorListener.cpp
 * @brief connector listener class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <rtm/ConnectorListener.h>

namespace RTC
{
  /*!
   * @if jp
   * @class ConnectorDataListener クラス
   * @else
   * @class ConnectorDataListener class
   * @endif
   */
  ConnectorDataListener::~ConnectorDataListener(){}

  /*!
   * @if jp
   * @class ConnectorListener クラス
   * @else
   * @class ConnectorListener class
   * @endif
   */
  ConnectorListener::~ConnectorListener(){}

  /*!
   * @if jp
   * @class ConnectorDataListener ホルダクラス
   * @else
   * @class ConnectorDataListener holder class
   * @endif
   */
  ConnectorDataListenerHolder::ConnectorDataListenerHolder()
  {
  }
  

  ConnectorDataListenerHolder::~ConnectorDataListenerHolder()
  {
    Guard guard(m_mutex);
    for (int i(0), len(m_listeners.size()); i < len; ++i)
      {
        if (m_listeners[i].second)
          {
            delete m_listeners[i].first;
          }
      }
  }

  
  void ConnectorDataListenerHolder::
  addListener(ConnectorDataListener* listener, bool autoclean)
  {
    Guard guard(m_mutex);
    m_listeners.push_back(Entry(listener, autoclean));
  }

  
  void ConnectorDataListenerHolder::
  removeListener(ConnectorDataListener* listener)
  {
    Guard guard(m_mutex);
    std::vector<Entry>::iterator it(m_listeners.begin());
    for (; it != m_listeners.end(); ++it)
      {
        if ((*it).first == listener)
          {
            if ((*it).second)
              {
                delete (*it).first;
              }
            m_listeners.erase(it);
            return;
          }
      }
    
  }

    
  void ConnectorDataListenerHolder::notify(const ConnectorInfo& info,
                                           const cdrMemoryStream& cdrdata)
  {
    Guard guard(m_mutex);
    for (int i(0), len(m_listeners.size()); i < len; ++i)
      {
        m_listeners[i].first->operator()(info, cdrdata);
      }
  }


  /*!
   * @if jp
   * @class ConnectorListener ホルダクラス
   * @else
   * @class ConnectorListener holder class
   * @endif
   */
  ConnectorListenerHolder::ConnectorListenerHolder()
  {
  }
    
  
  ConnectorListenerHolder::~ConnectorListenerHolder()
  {
    Guard guard(m_mutex);
    for (int i(0), len(m_listeners.size()); i < len; ++i)
      {
        if (m_listeners[i].second)
          {
            delete m_listeners[i].first;
          }
      }
  }

  
  void ConnectorListenerHolder::addListener(ConnectorListener* listener,
                                            bool autoclean)
  {
    Guard guard(m_mutex);
    m_listeners.push_back(Entry(listener, autoclean));
  }
  

  void ConnectorListenerHolder::removeListener(ConnectorListener* listener)
  {
    Guard guard(m_mutex);
    std::vector<Entry>::iterator it(m_listeners.begin());
    
    for (; it != m_listeners.end(); ++it)
      {
        if ((*it).first == listener)
          {
            if ((*it).second)
              {
                delete (*it).first;
              }
            m_listeners.erase(it);
            return;
          }
      }
    
  }
  
  
  void ConnectorListenerHolder::notify(const ConnectorInfo& info)
  {
    Guard guard(m_mutex);
    for (int i(0), len(m_listeners.size()); i < len; ++i)
      {
        m_listeners[i].first->operator()(info);
      }
  }
};


