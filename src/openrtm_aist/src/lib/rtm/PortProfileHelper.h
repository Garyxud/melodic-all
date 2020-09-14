// -*- C++ -*-
/*!
 * @file PortProfileHelper.h
 * @brief RTC's PortProfile helper class
 * @date $Date: 2007-04-26 15:32:25 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef RTC_PORTPROFILEHELPER_H
#define RTC_PORTPROFILEHELPER_H


// RTC header include
#include <rtm/RTC.h>
#include <rtm/Util.h>

// ACE includes
#include <coil/Mutex.h>

// CORBA header include
#include <rtm/idl/RTCSkel.h>




namespace RTC
{
  /*!
   * @if jp
   *
   * @class PortProfileHelper
   * @brief PortProfile ヘルパークラス
   *
   * RTC::Port の種々のプロファイルを保持する PortProfile を管理するクラス。
   * 主として PortBase の内部で使用される。
   *
   * @else
   *
   * @class PortProfileHelper
   * @brief PortProfile helper class
   *
   * This class manages the PortProfile that is profiles of the RTC:Port.
   * This is mainly used in PortBase class.
   *
   * @endif
   */
  class PortProfileHelper
  {
    typedef coil::Mutex Mutex;

  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @else
     * @brief Constructor
     *
     * Constructor
     *
     * @endif
     */
    PortProfileHelper();
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    virtual ~PortProfileHelper(void);


    /*!
     * @if jp
     *
     * @brief PortProfile を設定する
     *
     * このオブジェクトが保持する PortProfile を引数で与えられた PortProfile
     * をコピーし上書きして保存する。
     *
     * @param PortProfile 上書きする PortProfile
     *
     * @else
     *
     * @brief Set PortProfile
     *
     * This operation copies the given PortProfile and overwrites the existent
     * PortProfile by the given ProtProfile.
     *
     * @param PortProfile The PortProfile to be stored.
     *
     * @endif
     */
    void setPortProfile(const PortProfile& profile);


    /*!
     * @if jp
     *
     * @brief PortProfile を取得する
     *
     * このオブジェクトが保持する PortProfile を返す。
     *
     * @return このオブジェクトが保持する PortProfile
     *
     * @else
     *
     * @brief Get PortProfile
     *
     * This operation returns the PortProfile.
     *
     * @return The PortProfile stored by the object.
     *
     * @endif
     */
    PortProfile* getPortProfile();


    /*!
     * @if jp
     *
     * @brief PortProfile.name を設定する
     *
     * このオペレーションは引数で与えられた文字列をコポーし、
     * PortProfile.name として保持する。
     *
     * @param name PortProfile.name に格納する Port の名前
     *
     * @else
     *
     * @brief Set PortProfile.name
     *
     * This operation stores a copy of given name to the PortProfile.name.
     *
     * @param name The name of Port to be stored to the PortProfile.name.
     *
     * @endif
     */
    void setName(const char* name);


    /*!
     * @if jp
     *
     * @brief PortProfile.name を取得する
     *
     * このオペレーションは PortProfile.name を取得する。
     *
     * @return PortProfile.name へのポインタ
     *
     * @else
     *
     * @brief Get PortProfile.name
     *
     * This operation returns a pointer to the PortProfile.name.
     *
     * @return The pointer to PortProfile.name.
     *
     * @endif
     */
    const char* getName() const;


    /*!
     * @if jp
     *
     * @brief PortInterfaceProfile を追加する
     *
     * このオペレーションは PortProfile に PortInterfaceProfile を追加する。
     *
     * @param if_profile PortProfile に追加する PortInterfaceProfile
     *
     * @else
     *
     * @brief Append PortInterfaceProfile to the PortProfile
     *
     * This operation appends the PortInterfaceProfile to the PortProfile
     *
     * @param if_profile PortInterfaceProfile to be appended the PortProfile
     *
     * @endif
     */
    void appendPortInterfaceProfile(PortInterfaceProfile if_prof);


    /*!
     * @if jp
     *
     * @brief PortInterfaceProfileList を取得する
     *
     * このオペレーションは PortInterfaceProfileList を返す。
     *
     * @return PortInterfaceProfileList
     *
     * @else
     *
     * @brief Get PortInterfaceProfileList
     *
     * This operation returns the PortInterfaceProfileList.
     *
     * @return PortInterfaceProfileList
     *
     * @endif
     */
    const PortInterfaceProfileList& getPortInterfaceProfiles() const;


    /*!
     * @if jp
     *
     * @brief PortInterfaceProfile を取得する
     *
     * このオペレーションは instance_name で指定された PortInterfaceProfile
     * を返す。
     *
     * @param instance_name PortInterfaceProfile の instance_name
     * @return PortInterfaceProfile
     *
     * @else
     *
     * @brief Get PortInterfaceProfile
     *
     * This operation returns the PortInterfaceProfile specified
     * by instance_name.
     *
     * @param instance_name instance_name of the PortInterfaceProfile
     * @return PortInterfaceProfile
     *
     * @endif
     */
    const PortInterfaceProfile
    getPortInterfaceProfile(const char* instance_name) const;


    /*!
     * @if jp
     *
     * @brief PortInterfaceProfile を削除する
     *
     * このオペレーションは instance_name で指定された　PortInterfaceProfile
     * を削除する。指定した名前の PortInterfaceProfile が存在しない場合には、
     * NotFound exception を返す。
     *
     * @param instance_name 削除する PortInterfaceProfile の名前
     *
     * @else
     *
     * @brief Erase PortInterfaceProfile from the PortProfile
     *
     * This operation erases the PortInterfaceProfile from the PortProfile
     *
     * @param instance_name PortInterfaceProfile to be erased from the
     *        PortProfile
     *
     * @endif
     */
    void erasePortInterfaceProfile(const char* instance_name);


    /*!
     * @if jp
     *
     * @brief Port のオブジェクト参照をセットする
     *
     * このオペレーションは PortProfile に、関連する Port のオブジェクト参照
     * を設定する。
     *
     * @param port 設定する Port のオブジェクトリファレンス
     *
     * @else
     *
     * @brief Set Port's object reference
     *
     * This operation set the object reference of the Port.
     *
     * @param port Port's object reference to be set.
     *
     * @endif
     */
    void setPortRef(PortService_ptr port);


    /*!
     * @if jp
     *
     * @brief Port のオブジェクト参照を取得する
     *
     * このオペレーションは PortProfile に関連付けられた Port の
     * オブジェクト参照を返す。
     *
     * @return 関連付けられた Port のオブジェクト参照
     *
     * @else
     *
     * @brief Get Port's object reference
     *
     * This operation returns the object reference of the PortProfile.
     *
     * @return Port's object reference associated with the PortProfile.
     *
     * @endif
     */
    PortService_ptr getPortRef() const;


    /*!
     * @if jp
     *
     * @brief ConnectorProfile を追加する
     *
     * このオペレーションは PortProfile に ConnectorProfile を追加する。
     *
     * @param conn_profile ConnectorProfile 
     *
     * @else
     *
     * @brief Append ConnectorProfile
     *
     * This operation appends the ConnectorProfile to the PortProfile.
     *
     * @param conn_profile ConnectorProfile to be added.
     *
     * @endif
     */
    void appendConnectorProfile(ConnectorProfile conn_profile);


    /*!
     * @if jp
     *
     * @brief ConnectorProfileList を取得する
     *
     * このオペレーションは PortProfile に関連付けられた ConnectorProfile の
     * リスト ConnectorProfileList を返す。
     *
     * @return 関連付けられた ConnectorProfileList
     *
     * @else
     *
     * @brief Get ConnectorProfileList
     *
     * This operation returns the list of ConnectorProfile of the PortProfile.
     *
     * @return Port's ConnectorProfileList.
     *
     * @endif
     */
    const ConnectorProfileList getConnectorProfiles() const;


    /*!
     * @if jp
     *
     * @brief ConnectorProfile を取得する
     *
     * このオペレーションは引数で指定された名前を持つ ConnectorProfile を返す。
     *
     * @param name ConnectorProfile の名前
     * @return ConnectorProfile
     *
     * @else
     *
     * @brief Get ConnectorProfile
     *
     * This operation returns the ConnectorProfile specified by name.
     *
     * @param name The name of ConnectorProfile
     * @return ConnectorProfile.
     *
     * @endif
     */
    const ConnectorProfile getConnectorProfile(const char* name) const;


     /*!
     * @if jp
     *
     * @brief ConnectorProfile を取得する
     *
     * このオペレーションは引数で指定されたIDを持つ ConnectorProfile を返す。
     *
     * @param id ConnectorProfile のID
     * @return ConnectorProfile
     *
     * @else
     *
     * @brief Get ConnectorProfile
     *
     * This operation returns the ConnectorProfile specified by ID.
     *
     * @param id The ID of ConnectorProfile
     * @return ConnectorProfile.
     *
     * @endif
     */
    const ConnectorProfile getConnectorProfileById(const char* id) const;


     /*!
     * @if jp
     *
     * @brief ConnectorProfile を削除する
     *
     * このオペレーションは PortProfile の ConnectorProfile を
     * 名前で指定して削除する。
     *
     * @param naem ConnectorProfile の名前
     *
     * @else
     *
     * @brief Erase ConnectorProfile
     *
     * This operation erases the ConnectorProfile from the PortProfile.
     *
     * @param name The name of the ConnectorProfile to be erased.
     *
     * @endif
     */
    void eraseConnectorProfile(const char* name);


     /*!
     * @if jp
     *
     * @brief ConnectorProfile を削除する
     *
     * このオペレーションは PortProfile の ConnectorProfile を
     * ID で指定して削除する。
     *
     * @param id ConnectorProfile のID
     *
     * @else
     *
     * @brief Erase ConnectorProfile
     *
     * This operation erases the ConnectorProfile from the PortProfile.
     *
     * @param id The ID of the ConnectorProfile to be erased.
     *
     * @endif
     */
    void eraseConnectorProfileById(const char* id);


     /*!
     * @if jp
     *
     * @brief PortProfile の owner を設定する
     *
     * このオペレーションは PortProfile の owner を設定する。
     *
     * @param owner PortProfile の owner のオブジェクト参照
     *
     * @else
     *
     * @brief Set owner's object reference to the PortProfile
     *
     * This operation sets the owner's object reference to the PortProfile.
     *
     * @param owner The owner's object reference of PortProfile.
     *
     * @endif
     */
    void setOwner(RTObject_ptr owner);


    /*!
     * @if jp
     *
     * @brief PortProfile の owner を取得する
     *
     * このオペレーションは PortProfile の owner のオブジェクト参照を返す。
     *
     * @return PortProfile の owner のオブジェクト参照
     *
     * @else
     *
     * @brief Get owner's object reference from the PortProfile
     *
     * This operation returns the owner's object reference of the PortProfile.
     *
     * @return The owner's object reference of PortProfile.
     *
     * @endif
     */
    RTObject_ptr getOwner() const;


    /*!
     * @if jp
     *
     * @brief PortProfile の properties を設定する
     *
     * このオペレーションは PortProfile に properties を設定する。
     *
     * @param prop PortProfile の properties の NVList
     *
     * @else
     *
     * @brief Set properties to the PortProfile
     *
     * This operation set the properties to the PortProfile.
     *
     * @param prop The NVList of PortProfile's properties.
     *
     * @endif
     */
    void setProperties(NVList& prop);


    /*!
     * @if jp
     *
     * @brief PortProfile の properties を取得する
     *
     * このオペレーションは PortProfile の propertiesを返す。
     *
     * @return PortProfile の properties の NVList
     *
     * @else
     *
     * @brief Get properties of the PortProfile
     *
     * This operation returns the properties of the PortProfile.
     *
     * @return The NVList of PortProfile's properties.
     *
     * @endif
     */
    const NVList& getProperties() const;



  private:
    // Specialization of SequenceEx template class


    // PortProfile.name
    std::string m_name;

    // PortProfile.interfaces
    typedef SequenceEx<PortInterfaceProfileList,
		       PortInterfaceProfile,
		       Mutex> IfProfiles;
    IfProfiles m_ifProfiles;

    // PortProfile.port_ref
    PortService_var m_portRef;

    // PortProfile.connector_profile
    typedef SequenceEx<ConnectorProfileList,
		       ConnectorProfile,
		       Mutex> ConnProfiles;
    ConnProfiles m_connProfiles ;

    // PortProfile.owner
    RTObject_var m_owner;

    // PortProfile.properties
    NVList m_properties;

    // Mutex
    mutable Mutex m_mutex;


    // Functor to find PortInterfaceProfile by name
    struct if_name
    {
      if_name(const char* name) :  m_name(name) {};
      bool operator()(const PortInterfaceProfile& p)
      {
	std::string name(p.instance_name);
	return m_name == name;
      }
      const std::string m_name;
    };

    // Functor to find ConnectorProfile by name
    struct conn_name
    {
      conn_name(const char* name) :  m_name(name) {};
      bool operator()(const ConnectorProfile& c)
      {
	std::string name(c.name);
	return m_name == name;
      }
      const std::string m_name;
    };
    
    // Functor to find ConnectorProfile by id
    struct conn_id
    {
      conn_id(const char* id) :  m_id(id) {};
      bool operator()(const ConnectorProfile& c)
      {
	std::string id(c.connector_id);
	return m_id == id;
      }
      const std::string m_id;
    };

  };   // class PortProfileHelper
};     // namespace RTC
#endif // RTC_PORTPROFILEHELPER_H
