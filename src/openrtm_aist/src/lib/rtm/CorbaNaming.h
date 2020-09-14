// -*- C++ -*-
/*!
 * @file CorbaNaming.h
 * @brief CORBA naming service helper class
 * @date $Date: 2007-12-31 03:08:02 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
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

#ifndef RTC_CORBANAMING_H
#define RTC_CORBANAMING_H

#include <rtm/RTC.h>

// STL includes
#include <map>
#include <string>
#include <vector>

/*!
 * @if jp
 * @namespace RTC
 *
 * @brief RTコンポーネント
 *
 * @else
 *
 * @namespace RTC
 *
 * @brief RT-Component
 *
 * @endif
 */

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace RTC
{
  /*!
   * @if jp
   * @class CorbaNaming
   * @brief CORBA Naming Service ヘルパークラス
   *
   * このクラスは、CosNaming::NamingContext に対するラッパークラスである。
   * CosNaming::NamingContext が持つオペレーションとほぼ同じ機能の
   * オペレーションを提供するとともに、ネームコンポーネント CosNaming::Name
   * の代わりに文字列による名前表現を受け付けるオペレーションも提供する。
   *
   * オブジェクトは生成時、あるいは生成直後に CORBA ネームサーバに接続し
   * 以後、このネームサーバのルートコンテキストに対して種々のオペレーション
   * を処理する。
   * 深い階層のネーミングコンテキストの作成やオブジェクトのバインドにおいて、
   * 途中のコンテキストが存在しない場合でも、強制的にコンテキストをバインド
   * し目的のコンテキストやオブジェクトのバインドを行うこともできる。
   *
   * @since 0.4.0
   *
   * @else
   * @class CorbaNaming
   * @brief CORBA Naming Service helper class
   *
   * This class is a wrapper class of CosNaming::NamingContext.
   * Almost the same operations which CosNaming::NamingContext has are
   * provided, and some operation allows string naming representation of
   * context and object instead of CosNaming::Name.
   *
   * The object of the class would connect to a CORBA naming server at
   * the instantiation or immediately after instantiation.
   * After that the object invokes operations to the root context of it.
   * This class realizes forced binding to deep NamingContext, without binding
   * intermediate NamingContexts explicitly.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class CorbaNaming
  {
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * @param orb ORB
     *
     * @else
     *
     * @brief Consructor
     *
     * @param orb ORB
     *
     * @endif
     */
    CorbaNaming(CORBA::ORB_ptr orb);
    
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * @param orb ORB
     * @param name_server ネームサーバの名称
     *
     * @else
     *
     * @brief Consructor
     *
     * @param orb ORB
     * @param name_server Name of the name server
     *
     * @endif
     */
    CorbaNaming(CORBA::ORB_ptr orb, const char* name_server);
    
    /*!
     * @if jp
     *
     * @brief 仮想デストラクタ
     * 
     * @else
     * 
     * @brief Virtual destructor
     * 
     * @endif
     */
    virtual ~CorbaNaming(void){};
    
    /*!
     * @if jp
     *
     * @brief ネーミングサービスの初期化
     * 
     * 指定されたネームサーバ上のネーミングサービスを初期化します。
     * 
     * @param name_server ネームサーバの名称
     * 
     * @else
     *
     * @brief Initialize the Naming Service
     * 
     * Initialize the Naming Service on the specified name server.
     * 
     * @param name_server Name of the name server
     * 
     * @endif
     */
    void init(const char* name_server);

    bool isAlive();
    
    typedef CORBA::SystemException SystemException;
    typedef CosNaming::NamingContext::NotFound      NotFound;
    typedef CosNaming::NamingContext::CannotProceed CannotProceed;
    typedef CosNaming::NamingContext::InvalidName   InvalidName;
    typedef CosNaming::NamingContext::AlreadyBound  AlreadyBound;
    typedef CosNaming::NamingContext::NotEmpty      NotEmpty;
    typedef CosNaming::NamingContextExt::InvalidAddress InvalidAddress;
    typedef std::vector<CORBA::Object_ptr> ObjectList;
    
    /*!
     * @if jp
     *
     * @brief Object を bind する
     *
     * CosNaming::bind() とほぼ同等の働きをするが、常に与えられたネームサーバの
     * ルートコンテキストに対してbind()が呼び出される点が異なる。
     *
     * Name <name> と Object <obj> を当該 NamingContext 上にバインドする。
     * c_n が n 番目の NameComponent をあらわすとすると、
     * name が n 個の NameComponent から成るとき、以下のように扱われる。
     *
     * cxt->bind(<c_1, c_2, ... c_n>, obj) は以下の操作と同等である。
     * cxt->resolve(<c_1, ... c_(n-1)>)->bind(<c_n>, obj)
     *
     * すなわち、1番目からn-1番目のコンテキストを解決し、n-1番目のコンテキスト
     * 上に name <n> として　obj を bind する。
     * 名前解決に参加する <c_1, ... c_(n-1)> の NemingContext は、
     * bindContext() や rebindContext() で既にバインド済みでなければならない。
     * もし <c_1, ... c_(n-1)> の NamingContext が存在しない場合には、
     * NotFound 例外が発生する。
     *
     * ただし、強制バインドフラグ force が true の時は、<c_1, ... c_(n-1)>
     * が存在しない場合にも、再帰的にコンテキストをバインドしながら、
     * 最終的に obj を名前 name <c_n> にバインドする。
     *
     * いずれの場合でも、n-1番目のコンテキスト上に name<n> のオブジェクト
     * (Object あるいは コンテキスト) がバインドされていれば
     * AlreadyBound 例外が発生する。
     *
     * @param name オブジェクトに付ける名前の NameComponent
     * @param obj 関連付けられる Object
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <c_n> の Object がすでにバインドされている。
     *
     * @else
     *
     * @brief Bind object on specified name component position
     *
     * Almost the same operation as CosNaming::bind(), but there is a difference 
     * that bind() is invoked for the root context of the given name server.
     *
     * Bind between Name <name> and Object <obj> on this NamingContext.
     * If c_n indicates the n-th of NameComponent,
     * when name consists of n pieces of NameComponent, it is handled as 
     * follows. 
     *
     * cxt->bind(<c_1, c_2, ... c_n>, obj) is the same as the following
     * operation.
     * cxt->resolve(<c_1, ... c_(n-1)>)->bind(<c_n>, obj)
     *
     * In other word, resolve from the first to the (n-1)th context and bind 
     * obj as name<n> on the (n-1)th context.
     * NemingContext of <c_1, ... c_(n-1)> for resolving name must be already 
     * bound in bindContext() or rebindContext().
     * If NamingContext of <c_1, ... c_(n-1)> does not exist, NotFound excption
     * will occur.
     *
     * However, when flag of forced bind is true, even if <c_1, ... c_(n-1)> does
     * not exist, finally obj will be bound to name name <c_n> by binding to 
     * the context recursively.
     *
     * Even in any case, if the object of name<n> (Object or context) is bound
     * on the (n-1)th context, AlreadyBound exception will occur.
     *
     * @param name NameComponent of name applied to object
     * @param obj Object that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<c_n> is already bound.
     *
     * @endif
     */
    void bind(const CosNaming::Name& name, CORBA::Object_ptr obj,
	      const bool force = 1)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief Object を bind する
     *
     * Object を bind する際に与える名前が文字列表現であること以外は、bind()
     * と同じである。bind(toName(string_name), obj) と等価。
     *
     * @param string_name オブジェクトに付ける名前の文字列表現
     * @param obj 関連付けられるオブジェクト
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <n> の Object がすでにバインドされている。
     *
     * @else
     *
     * @brief Bind object on specified string name position
     *
     * This is the same as bind() except as the given name is string 
     * representation when Object is bound. 
     * bind(toName(string_name),obj) is the same. 
     *
     * @param string_name The string representation of name applied to object
     * @param obj Object that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<c_n> is already bound.
     *
     * @endif
     */
    void bindByString(const char* string_name, CORBA::Object_ptr obj,
		      const bool force = 1)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief 途中のコンテキストを再帰的に bind しながら Object を bind する
     *
     * context で与えられた NamingContext に対して、name で指定された
     * ネームコンポーネント <c_1, ... c_(n-1)> を NamingContext として
     * 解決しながら、名前 <c_n> に対して obj を bind する。
     * もし、<c_1, ... c_(n-1)> に対応する NamingContext がない場合には
     * 新たな NamingContext をバインドする。
     *
     * 最終的に <c_1, c_2, ..., c_(n-1)> に対応する NamingContext が生成
     * または解決された上で、CosNaming::bind(<c_n>, object) が呼び出される。
     * このとき、すでにバインディングが存在すれば AlreadyBound例外が発生する。
     *
     * 途中のコンテキストを解決する過程で、解決しようとするコンテキストと
     * 同じ名前の NamingContext ではない Binding が存在する場合、
     * CannotProceed 例外が発生し処理を中止する。
     *
     * @param context bind を開始する　NamingContext
     * @param name オブジェクトに付ける名前のネームコンポーネント
     * @param obj 関連付けられるオブジェクト
     *
     * @exception CannotProceed <c_1, ..., c_(n-1)> に対応する NamingContext 
     *            のうちひとつが、すでに NamingContext 以外の object にバインド
     *            されており、処理を継続できない。
     * @exception InvalidName 名前 name が不正
     * @exception AlreadyBound name <c_n> にすでに何らかの object がバインド
     *            されている。
     * @else
     *
     * @brief Bind intermediate context recursively and bind object
     *
     * For NamingContext given in context, bind obj to name <c_n> with solving
     * name component <c_1, ... c_(n-1)> specified by name as NamingContext.
     * Bind new NamingContext when there is no NamingContext corresponding to
     * c_(n-1) >.
     *
     * Finally, NamingContext corresponding to <c_1, c_2, ..., c_(n-1)> 
     * will be generated, or CosNaming::bind(<c_n>, object) will be invoked
     * after solving. At this time, if the binding already exists, 
     * the AlreadyBound exception will occur.
     *
     * During process, when Binding that is not NamingContext of the same name
     * as the context for solving exists, CannotProceed exception will occur
     * and stop processing.
     *
     * @param context NamingContext that starts the bind
     * @param name NameComponent of name applied to object
     * @param obj Object that is associated
     *
     * @exception CannotProceed Since one of NamingContext corresponding to
     *                          <c_1, ..., c_(n-1)> is already bound to object
     *                          other than NamingContext and processing cannot 
     *                          be continued
     * @exception InvalidName name 'name' is invalid.
     * @exception AlreadyBound The object of name<c_n> is already bound.
     *
     * @endif
     */
    void bindRecursive(CosNaming::NamingContext_ptr context,
		       const CosNaming::Name& name,
		       CORBA::Object_ptr obj)
      throw (SystemException, CannotProceed, InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief Object を rebind する
     *
     * name で指定された Binding がすでに存在する場合を除いて bind() と同じ
     * である。バインディングがすでに存在する場合には、新しいバインディングに
     * 置き換えられる。
     *
     * @param name オブジェクトに付ける名前の NameComponent
     * @param obj 関連付けられるオブジェクト
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 名前 name が不正
     *
     * @else
     *
     * @brief Rebind object
     *
     * This is the same as bind() except as Binding specified by name 
     * already exists. If the binding already exists, new binding will be 
     * replaced.
     *
     * @param name NameComponent of name applied to object
     * @param obj Object that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName Name 'name' is invalid.
     *
     * @endif
     */
    void rebind(const CosNaming::Name& name, CORBA::Object_ptr obj,
		const bool force = 1)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief Object を rebind する
     *
     * Object を rebind する際に与える名前が文字列表現であること以外は rebind()
     * と同じである。rebind(toName(string_name), obj) と等価。
     *
     * @param string_name オブジェクトに付ける名前の文字列表現
     * @param obj 関連付けられるオブジェクト
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Rebind Object
     *
     * This is the same as rebind() except as the given name is string
     * representation when object is rebound. rebind(toName(string_name), obj) 
     * is the same.
     *
     * @param string_name NameComponent of name applied to object
     * @param obj Object that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    void rebindByString(const char* string_name, CORBA::Object_ptr obj,
			const bool force = 1)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief 途中のコンテキストを bind しながら Object を rebind する
     *
     * name <c_n> で指定された NamingContext もしくは Object がすでに存在する
     * 場合を除いて bindRecursive() と同じである。
     *
     * name <c_n> で指定されたバインディングがすでに存在する場合には、
     * 新しいバインディングに置き換えられる。
     *
     * @param context bind を開始する　NamingContext
     * @param name オブジェクトに付ける名前の NameComponent
     * @param obj 関連付けられるオブジェクト
     *
     * @exception CannotProceed 途中のコンテキストが解決できない。
     * @exception InvalidName 与えられた name が不正。
     *
     * @else
     *
     * @brief Bind intermediate context recursively and rebind object
     *
     * This is the same as bindRecursive() except as NamingContext 
     * or Object specified by name <c_n> already exists.
     *
     * If the binding specified by name <c_n> already exists, 
     * new binding will be replaced.
     *
     * @param context NamingContext that starts the bind
     * @param name NameComponent of name applied to object
     * @param obj Object that is associated
     *
     * @exception CannotProceed The intermediate context cannot resolved.
     * @exception InvalidName The given name is invalid.
     *
     * @endif
     */
    void rebindRecursive(CosNaming::NamingContext_ptr context,
			 const CosNaming::Name& name,
			 CORBA::Object_ptr obj)
      throw (SystemException, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief NamingContext を bind する
     *
     * bind されるオブジェクトが NamingContext であることを除いて bind() 
     * と同じである。
     *
     * @param name オブジェクトに付ける名前のネームコンポーネント
     * @param name_cxt 関連付けられる NamingContext
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <c_n> の Object がすでにバインドされている。
     *
     * @else
     *
     * @brief Bind NamingContext
     *
     * This is the same as bind() except as the bound object is NamingContext.
     *
     * @param name NameComponent of name applied to object
     * @param name_cxt Object that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<c_n> is already bound.
     *
     * @endif
     */
    void bindContext(const CosNaming::Name& name,
		     CosNaming::NamingContext_ptr name_cxt,
		     const bool force = 1)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief NamingContext を bind する
     *
     * bind されるオブジェクトが NamingContext であることを除いて
     * bindByString() と同じである。
     *
     * @param string_name オブジェクトに付ける名前の文字列表現
     * @param name_cxt 関連付けられる NamingContext
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <n> の Object がすでにバインドされている。
     *
     * @else
     *
     * @brief Bind NamingContext
     *
     * This is the same as bindByString() except as the bound object is
     * NamingContext.
     *
     * @param string_name String representation of name applied to object
     * @param name_cxt NamingContext that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<n> is already bound.
     *
     * @endif
     */
    void bindContext(const char* string_name,
		     CosNaming::NamingContext_ptr name_cxt,
		     const bool force = 1)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief 途中のコンテキストを再帰的に bind し NamingContext を bind する
     *
     * bind されるオブジェクトが NamingContext であることを除いて
     * bindRecursive() と同じである。
     *
     * @param context bind を開始する　NamingContext
     * @param name オブジェクトに付ける名前のネームコンポーネント
     * @param name_cxt 関連付けられる NamingContext
     *
     * @else
     *
     * @brief Bind intermediate context recursively and bind NamingContext
     *
     * This is the same as bindRecursive() except as the bound object
     * is NamingContext.
     *
     * @param context NamingContext that starts the bind
     * @param name NameComponent of name applied to object
     * @param name_cxt NamingContext that is associated
     *
     * @endif
     */
    void bindContextRecursive(CosNaming::NamingContext_ptr context,
			      const CosNaming::Name& name,
			      CosNaming::NamingContext_ptr name_cxt);
    
    /*!
     * @if jp
     *
     * @brief NamingContext を rebind する
     *
     * name で指定されたコンテキストがすでに存在する場合を除いて bindContext() 
     * と同じである。
     * バインディングがすでに存在する場合には、新しいバインディングに
     * 置き換えられる。
     *
     * @param name オブジェクトに付ける名前のネームコンポーネント
     * @param name_cxt 関連付けられる NamingContext
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Rebind NamingContext
     *
     * This is the same as bindContext() except as context specified
     * by name already exists.
     * If the binding already exists, new binding will be replaced.
     *
     * @param name NameComponent applied to object
     * @param name_cxt Object that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName the argument 'name' is invalid.
     *
     * @endif
     */
    void rebindContext(const CosNaming::Name& name,
		       CosNaming::NamingContext_ptr name_cxt,
		       const bool force = 1)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief NamingContext を rebind する
     *
     * name で指定されたコンテキストがすでに存在する場合を除いて bindContext() 
     * と同じである。
     * バインディングがすでに存在する場合には、新しいバインディングに
     * 置き換えられる。
     *
     * @param string_name オブジェクトに付ける名前の文字列表現
     * @param name_cxt 関連付けられる NamingContext
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Rebind NamingContext
     *
     * This is the same as bindContext() except as context specified
     * by name already exists.
     * If the binding already exists, new binding will be replaced.
     *
     * @param string_name String representation of name applied to object
     * @param name_cxt NamingContext that is associated
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    void rebindContext(const char* string_name,
		       CosNaming::NamingContext_ptr name_cxt,
		       const bool force = 1)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief 途中のコンテキストを再帰的に rebind し NamingContext を rebind する
     *
     * bind されるオブジェクトが NamingContext であることを除いて
     * rebindRecursive() と同じである。
     *
     * @param context bind を開始する　NamingContext
     * @param name オブジェクトに付ける名前の NameComponent
     * @param name_cxt 関連付けられる NamingContext
     *
     * @else
     *
     * @brief Rebind intermediate context recursively and rebind NamingContext
     *
     * This is the same as rebindRecursive() except as the bound object is 
     * NamingContext.
     *
     * @param context NamingContext that starts the bind
     * @param name NameComponent applied to object
     * @param name_cxt NamingContext that is associated
     *
     * @endif
     */
    void rebindContextRecursive(CosNaming::NamingContext_ptr context,
				const CosNaming::Name& name,
				CosNaming::NamingContext_ptr name_cxt);
    
    /*!
     * @if jp
     *
     * @brief 与えられた NameComponent にバインドされている Object を返す
     *
     * name に bind されているオブジェクト参照を返す。
     * ネームコンポーネント <c_1, c_2, ... c_n> は再帰的に解決される。
     * 
     * CosNaming::resolve() とほぼ同等の働きをするが、常に与えられた
     * ネームサーバのルートコンテキストに対して resolve() が呼び出される点が
     * 異なる。
     *
     * @param name 解決すべきオブジェクトの名前のネームコンポーネント
     *
     * @return 解決されたオブジェクト参照
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Return object bound on the specified NameComponent
     *
     * Return the object reference that is bound to name.
     * Resolve the name component<c_1, c_2, ... c_n> recursively.
     * 
     * Almost the same operation as CosNaming::resolve(), 
     * but there is a difference that resolve() is invoked for the root context
     * of the given name server.
     *
     * @param name The name component of object name that should be resolved
     *
     * @return The reference to the resolved object
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    CORBA::Object_ptr resolve(const CosNaming::Name& name)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief 与えられた NameComponent にバインドされている Object を返す
     *
     * name に bind されているオブジェクト参照を返す。
     * ネームコンポーネント <c_1, c_2, ... c_n> は再帰的に解決される。
     * 
     * CosNaming::resolve() とほぼ同等の働きをするが、常に与えられた
     * ネームサーバのルートコンテキストに対して resolve() が呼び出される点が
     * 異なる。
     *
     * @param string_name 解決すべきオブジェクトの名前の文字列表現
     *
     * @return 解決されたオブジェクト参照
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Return object bound on the specified name
     *
     * Return the object reference that is bound to name.
     * Resolve the name component<c_1, c_2, ... c_n> recursively.
     * 
     * Almost the same operation as CosNaming::resolve(), 
     * but there is a difference that resolve() is invoked for the root context
     * of the given name server.
     *
     * @param string_name The string representation of object name 
     *             that should be resolved
     *
     * @return The reference to the resolved object
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    CORBA::Object_ptr resolve(const char* string_name)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief 与えられた NameComponent のバインディングを削除する
     *
     * name に bind されているオブジェクト参照を返す。
     * ネームコンポーネント <c_1, c_2, ... c_n> は再帰的に解決される。
     * 
     * CosNaming::unbind() とほぼ同等の働きをするが、常に与えられた
     * ネームサーバのルートコンテキストに対して unbind() が呼び出される点が
     * 異なる。
     *
     * @param name 削除するオブジェクトのネームコンポーネント
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Unbind a binding specified by NameComponent
     *
     * Return the object reference that is bound to name.
     * Resolve the name component<c_1, c_2, ... c_n> recursively.
     * 
     * Almost the same operation as CosNaming::unbind(), 
     * but there is a difference that unbind() is invoked for the root context
     * of the always given name server.
     *
     * @param name The name component of the deleted object
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    void unbind(const CosNaming::Name& name)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief 与えられた NameComponent のバインディングを削除する
     *
     * name に bind されているオブジェクト参照を返す。
     * ネームコンポーネント <c_1, c_2, ... c_n> は再帰的に解決される。
     * 
     * CosNaming::unbind() とほぼ同等の働きをするが、常に与えられた
     * ネームサーバのルートコンテキストに対して unbind() が呼び出される点が
     * 異なる。
     *
     * @param string_name 解決すべきオブジェクトの名前の文字列表現
     *
     * @return 解決されたオブジェクト参照
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Unbind a binding specified by string representation
     *
     * Return the object reference that is bound to name.
     * Resolve the name component<c_1, c_2, ... c_n> recursively.
     * 
     * Almost the same operation as CosNaming::unbind(), 
     * but there is a difference that unbind() is invoked for the root context
     * of the always given name server.
     *
     * @param string_name The string representation of object name
     *                    that should be resolved
     *
     * @return The resolved object reference
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    void unbind(const char* string_name)
      throw (SystemException, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     *
     * @brief 新しいコンテキストを生成する
     *
     * 与えられたネームサーバ上で生成された NamingContext を返す。
     * 返された NamingContext は bind されていない。
     * 
     * @return 生成された新しい NamingContext
     *
     * @else
     *
     * @brief Create new NamingContext
     *
     * Return NamingContext that has been created on the given name server.
     * The returned NamingContext has not bound yet.
     * 
     * @return New created NamingContext
     *
     * @endif
     */
    CosNaming::NamingContext_ptr newContext();
    
    /*!
     * @if jp
     *
     * @brief 新しいコンテキストを bind する
     *
     * 与えられた name に対して新しいコンテキストをバインドする。
     * 生成された　NamingContext はネームサーバ上で生成されたものである。
     * 
     * @param name NamingContextに付ける名前のネームコンポーネント
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     *
     * @return 生成された新しい NamingContext
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <n> の Object がすでにバインドされている。
     *
     * @else
     *
     * @brief Bind new NamingContext
     *
     * Bind new context for the given name.
     * The created NamingContext is a creation on the name server.
     * 
     * @param name NameComponent applied to NamingContext
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     *
     * @return New created NamingContext
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<n> is already bound.
     *
     * @endif
     */
    CosNaming::NamingContext_ptr
    bindNewContext(const CosNaming::Name& name, bool force = true)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief 新しいコンテキストを bind する
     *
     * 与えられた文字列に対応する新しいコンテキストをバインドする。
     * 生成された　NamingContext はネームサーバ上で生成されたものである。
     * 
     * @param string_name NamingContextに付ける名前の文字列表現
     * @param force trueの場合、途中のコンテキストを強制的にバインドする
     *              (デフォルト値:true)
     * 
     * @return 生成された新しい NamingContext
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <n> の Object がすでにバインドされている。
     *
     * @else
     *
     * @brief Bind new NamingContext
     *
     * Bind new context corresponding to the given string.
     * The created NamingContext is a creation on the name server.
     * 
     * @param string_name The string representation of name applied to 
     *                    NamingContext
     * @param force If true, the intermediate context is bound forcibly.
     *              (The default value:true)
     * 
     * @return New created NamingContext
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<n> is already bound.
     *
     * @endif
     */
    CosNaming::NamingContext_ptr
    bindNewContext(const char* string_name, bool force = true)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    /*!
     * @if jp
     *
     * @brief NamingContext を非アクティブ化する
     *
     * context で指定された NamingContext を非アクティブ化する。
     * context に他のコンテキストがバインドされている場合は NotEmpty 例外が
     * 発生する。
     * 
     * @param context 非アクティブ化する NamingContext
     *
     * @exception NotEmpty 対象context に他のコンテキストがバインドされている。
     *
     * @else
     *
     * @brief Destroy the naming context
     *
     * Destroy the specified naming context.
     * Any bindings should be <unbind> in which the given context is bound to
     * some names before invoking <destroy> operation on it. 
     *
     * @param context NamingContext which is destroied.
     *     
     * @exception NotEmpty The target context is bound to the other context.
     *
     * @endif
     */
    void destroy(CosNaming::NamingContext_ptr context)
      throw (SystemException, NotEmpty);
    
    /*!
     * @if jp
     * @brief NamingContext を再帰的に下って非アクティブ化する
     *
     * context で与えられた NamingContext に対して、name で指定された
     * ネームコンポーネント <c_1, ... c_(n-1)> を NamingContext として
     * 解決しながら、名前 <c_n> に対して 非アクティブ化を行う。
     *
     * @param context 非アクティブ化する NamingContext
     *
     * @exception NotEmpty 対象context に他のコンテキストがバインドされている。
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     *
     * @brief Destroy the naming context recursively
     *
     * For NamingContext given by Context, Destroy name <c_n> with 
     * solving the name component specified by name as NamingContext recursively.
     *
     * @param context NamingContext which is Destroied.
     *
     * @exception NotEmpty The target context is bound to the other context.
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    void destroyRecursive(CosNaming::NamingContext_ptr context)
      throw (SystemException, NotEmpty, NotFound, CannotProceed, InvalidName);
    
    /*!
     * @if jp
     * @brief すべての Binding を削除する
     *
     * 登録されている全てのBinding を削除する。
     *
     * @else
     * @brief Destroy all bindings
     *
     * Destroy all bindings that are registered.
     *
     * @endif
     */
    void clearAll();
    
    /*!
     * @if jp
     * @brief 与えられた NamingContext の Binding を取得する
     *
     * 指定された NamingContext の Binding を取得する。
     *
     * @param name_cxt Binding 取得対象 NamingContext
     * @param how_many Binding を取得する階層の深さ
     * @param bl 取得した Binding を保持するホルダ
     * @param bi 取得した Binding をたどるためのイテレータ
     *
     * @else
     * @brief Get Binding of the given NamingContext
     *
     * Get Binding of the given NamingContext.
     *
     * @param name_cxt NamingContext of the getting target Binding
     * @param how_many The depth to get Binding
     * @param bl The holder to hold the got Binding
     * @param bi The iterator to detect the got Binding
     *
     * @endif
     */
    void list(CosNaming::NamingContext_ptr name_cxt,
	      CORBA::ULong how_many,
	      CosNaming::BindingList_var& bl,
	      CosNaming::BindingIterator_var& bi);
    
    //============================================================
    // interface of NamingContextExt
    //============================================================
    /*!
     * @if jp
     * @brief 与えられた NameComponent の文字列表現を返す
     *
     * 指定された NameComponent を文字に変換する。
     *
     * @param name 変換対象 NameComponent
     *
     * @return 文字列変換結果
     *
     * @exception InvalidName 引数 name の名前が不正。
     *
     * @else
     * @brief Get string representation of given NameComponent
     *
     * Transform specified NameComponent into string representation.
     *
     * @param name The target NameComponent for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    char* toString(const CosNaming::Name& name)
      throw (SystemException, InvalidName);
    
    /*!
     * @if jp
     * @brief 与えられた文字列表現を NameComponent に分解する
     *
     * 指定された文字列を NameComponent に変換する。
     *
     * @param string_name 変換対象文字列
     *
     * @return NameComponent 変換結果
     *
     * @exception InvalidName 引数 string_name が不正。
     *
     * @else
     * @brief Resolve given string representation to NameComponent
     *
     * Transform given string representation to NameComponent.
     *
     * @param string_name The target string representation to transform
     *
     * @return NameComponent The result of transformation
     *
     * @exception InvalidName The argument 'name' is invalid.
     *
     * @endif
     */
    CosNaming::Name toName(const char* string_name)
      throw (SystemException, InvalidName);
    
    /*!
     * @if jp
     * @brief 与えられた addre と string_name から URL表現を取得する
     *
     * 指定されたアドレスと名称をURLに変換する。
     *
     * @param addr 変換対象アドレス
     * @param string_name 変換対象名称
     *
     * @return URL 変換結果
     *
     * @exception InvalidAddress 引数 addr が不正。
     * @exception InvalidName 引数 string_name が不正。
     *
     * @else
     * @brief Get URL representation from given addr and string_name
     *
     * Convert specified addr and string_name into URL
     *
     * @param addr The target address for conversion
     * @param string_name The target name for conversion
     *
     * @return URL Conversion result
     *
     * @exception InvalidAddress The argument 'addr' is invalid.
     * @exception InvalidName The argument 'string_name' is invalid.
     *
     * @endif
     */
    char* toUrl(char* addr, char* string_name)
      throw (SystemException, InvalidAddress, InvalidName);
    
    /*!
     * @if jp
     * @brief 与えられた文字列表現を resolve しオブジェクトを返す
     *
     * 指定された文字列表現をresolveし，オブジェクトを取得する。
     *
     * @param string_name 取得対象オブジェクト文字列表現
     *
     * @return 解決されたオブジェクト
     *
     * @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
     * @exception CannotProceed 何らかの理由で処理を継続できない。
     * @exception InvalidName 引数 name の名前が不正。
     * @exception AlreadyBound name <n> の Object がすでにバインドされている。
     *
     * @else
     * @brief Resolve from name of string representation and get object
     *
     * Resolve specified string representation and get object
     *
     * @param string_name The string representation of getting target object
     *
     * @return The resolved object
     *
     * @exception NotFound There is not <c_1, c_2, ..., c_(n-1)>.
     * @exception CannotProceed Processing cannot be continued for some reasons.
     * @exception InvalidName The argument 'name' is invalid.
     * @exception AlreadyBound The object of name<n> is already bound.
     *
     * @endif
     */
    CORBA::Object_ptr resolveStr(const char* string_name)
      throw (SystemException, NotFound, CannotProceed,
             InvalidName, AlreadyBound);
    
    //============================================================
    // Find functions
    //============================================================
    
    //    ObjectList find(const char* name, const char* kind);
    //    ObjectList findById(const char* name, const char* kind);
    //    ObjectList findByKind(const char* name, const char* kind);
    
    /*!
     * @if jp
     *
     * @brief オブジェクトの名前をバインドまたは解決する
     *
     * 指定されたコンテキストに対してオブジェクトを NameComponent で指定された
     * 位置にバインドする。
     * 同一箇所に既に他の要素がバインド済みの場合は、既存のバインド済み要素を
     * 取得する。
     *
     * @param context bind もしくは resole 対象コンテキスト
     * @param name オブジェクトに付ける名前の NameComponent
     * @param obj 関連付けられる Object
     *
     * @return NameComponent で指定された位置にバインドされているオブジェクト
     *
     * @else
     * @brief Bind or resolve the given name component
     *
     * Bind object at the position that specified in NameComponent for the 
     * specified context.
     * When other elements are already bound at the same position, get the 
     * already bound element.
     *
     * @param context The context to bind or resole
     * @param name NameComponent applied to object
     * @param obj Object that is associated
     *
     * @return The object that is bound at position specified with NameComponent
     *
     * @endif
     */
    CORBA::Object_ptr bindOrResolve(CosNaming::NamingContext_ptr context,
				    const CosNaming::Name& name,
				    CORBA::Object_ptr obj);
    
    /*!
     * @if jp
     *
     * @brief 名前をバインドまたは解決する
     *
     * 指定されたコンテキストに対して Contextを NameComponent で指定された位置に
     * バインドする。
     * 同一箇所に既に他の要素がバインド済みの場合は、既存のバインド済み要素を
     * 取得する。
     *
     * @param context bind もしくは resole 対象コンテキスト
     * @param name コンテキストに付ける名前の NameComponent
     * @param new_context 関連付けられる Context
     *
     * @return NameComponent で指定された位置にバインドされているContext
     *
     * @else
     * @brief Bind or resolve the given name component
     *
     * Bind Context at the position that specified in NameComponent for the 
     * specified context.
     * When other elements are already bound at the same position, get the 
     * already bound element.
     *
     * @param context The context to bind or resole
     * @param name NameComponent applied to object
     * @param new_context Context that is associated
     *
     * @return The Context that is bound at the position specified with 
     *         NameComponent
     *
     * @endif
     */
    CosNaming::NamingContext_ptr
    bindOrResolveContext(CosNaming::NamingContext_ptr context,
			 const CosNaming::Name& name,
			 CosNaming::NamingContext_ptr new_context);
    
    /*!
     * @if jp
     * @brief 名前をバインドまたは解決する
     *
     * 指定されたコンテキストに対して NameComponent で指定された位置に
     * 新規コンテキストをバインドする。
     * 同一箇所に既に他の要素がバインド済みの場合は、既存のバインド済み要素を
     * 取得する。
     *
     * @param context bind もしくは resole 対象コンテキスト
     * @param name 新規作成するコンテキストの位置を表す NameComponent
     *
     * @return NameComponent で指定された位置にバインドされているContext
     *
     * @else
     * @brief Bind or resolve the given name component
     *
     * Bind new Context at the position that specified in NameComponent for the 
     * specified context.
     * When other elements are already bound at the same position, get the 
     * already bound element.
     *
     * @param context The context to bind or resole
     * @param name NameComponent that indicates the position of new context
     *
     * @return The Context that is bound at the position specified with 
     *         NameComponent
     *
     * @endif
     */
    CosNaming::NamingContext_ptr
    bindOrResolveContext(CosNaming::NamingContext_ptr context,
			 const CosNaming::Name& name);
    
    /*!
     * @if jp
     * @brief ネームサーバの名前を取得する
     *
     * 設定したネームサーバの名前を取得する。
     *
     * @return ネームサーバの名前
     *
     * @else
     * @brief Get the name of name server
     *
     * Get the configured name of name server
     *
     * @return The name of name server
     *
     * @endif
     */
    const char* getNameServer();
    
    /*!
     * @if jp
     * @brief ルートコンテキストを取得する
     *
     * 設定したネームサーバのルートコンテキストを取得する。
     *
     * @return ネームサーバのルートコンテキスト
     *
     * @else
     * @brief Get the root context
     *
     * Get the root context of the configured name server
     *
     * @return Root context ot name server
     *
     * @endif
     */
    CosNaming::NamingContext_ptr getRootContext();
    
    /*!
     * @if jp
     * @brief オブジェクトがネーミングコンテキストか判別する
     *
     * 指定した要素がネーミングコンテキストか判別する
     *
     * @param obj 判別対象要素
     *
     * @return 判別結果(ネーミングコンテキスト:true、それ以外:false)
     *
     * @else
     * @brief Determine whether the object is NamingContext
     *
     * Determine whether the specified element is NamingContext
     *
     * @param obj The target element for determination
     *
     * @return Determination result (NamingContext:true, Else:false)
     *
     * @endif
     */
    bool isNamingContext(CORBA::Object_ptr obj);
    
    /*!
     * @if jp
     * @brief 与えられた名前がネーミングコンテキストかどうか
     *
     * NameComponentで指定した要素がネーミングコンテキストか判別する
     *
     * @param name 判別対象NameComponent
     *
     * @return 判別結果(ネーミングコンテキスト:true、それ以外:false)
     *
     * @else
     * @brief Determine whether the given name component is NamingContext
     *
     * Determine whether the specified element is NameComponent
     *
     * @param name The target NameComponent for determination
     *
     * @return Determination result (NamingContext:true, Else:false)
     *
     * @endif
     */
    bool isNamingContext(const CosNaming::Name& name);
    
    /*!
     * @if jp
     * @brief 与えられた名前がネーミングコンテキストかどうか
     *
     * 文字列で指定した要素がネーミングコンテキストか判別する
     *
     * @param string_name 判別対象文字列
     *
     * @return 判別結果(ネーミングコンテキスト:true、それ以外:false)
     *
     * @else
     * @brief Determine whether the given string name is NamingContext
     *
     * Determine whether the element specified by string name is NamingContext
     *
     * @param string_name The string representation for determination
     *
     * @return Determination result (NamingContext:true, Else:false)
     *
     * @endif
     */
    bool isNamingContext(const char* string_name);
    
    /*!
     * @if jp
     * @brief ネームコンポーネントの部分を返す
     *
     * 指定された範囲のネームコンポーネントを取得する。
     * 終了位置が指定されていない場合は、最後の要素を除いたネームコンポーネント
     * を返す。
     *
     * @param name 検索対象NameComponent
     * @param begin 取得範囲開始位置
     * @param end 取得範囲終了位置(デフォルト値:-1)
     *
     * @return NameComponent 取得結果
     *
     * @else
     * @brief Get subset of given name component
     *
     * Get the name component in specified range.
     * Return the name component except the last element if the end 
     * position is not specified.
     *
     * @param name The target NameComponent for search
     * @param begin The beginning position for getting range
     * @param end The end position for getting range (The default value:-1)
     *
     * @return NameComponent Getting result
     *
     * @endif
     */
    CosNaming::Name subName(const CosNaming::Name& name,
			    CORBA::Long begin,
			    CORBA::Long end = -1);
    
  protected:
    /*!
     * @if jp
     * @brief ネームコンポーネントの文字列表現を取得する
     *
     * 指定した範囲のネームコンポーネントの文字列表現を取得する。
     * 文字列表現は、NameComponentの構成が{Nc[0], Nc[1], Nc[2]...}の場合、
     *   Nc[0]id.Nc[0].kind/Nc[1]id.Nc[1].kind/Nc[2].id/Nc[2].kind...
     * という形式で取得できる。
     * 取得した文字列の長さが指定した長さ以上の場合は、
     * 指定した長さで切り捨てられる。
     *
     * @param name 取得対象NameComponent
     * @param string_name 取得結果文字列
     * @param slen 取得対象文字列最大値
     *
     * @else
     * @brief Get string representation of name component
     *
     * Get string representation of the name component in specified range.
     * In string representation, if NameComponent consists of 
     * {Nc[0],Nc[1],Nc[2]...}, the format of 
     * Nc[0]id.Nc[0].kind/Nc[1]id.Nc[1].kind/Nc[2].id/Nc[2].kind...
     * will be got.
     * It is rounded by the specified length when the length of the got
     * string is over the specified length. 
     *
     * @param name The getting target NameComponent
     * @param string_name The string of getting result
     * @param slen The maximum length value of getting string
     *
     * @endif
     */
    void nameToString(const CosNaming::Name& name, char* string_name,
		      CORBA::ULong slen);
    /*!
     * @if jp
     * @brief ネームコンポーネントの文字列表現時の文字長を取得する
     *
     * 指定したネームコンポーネントを文字列で表現した場合の長さを取得する。
     * 文字列表現は、NameComponentの構成が{Nc[0],Nc[1],Nc[2]...}の場合、
     *   Nc[0]id.Nc[0].kind/Nc[1]id.Nc[1].kind/Nc[2].id/Nc[2].kind...
     * という形式で取得できる。
     *
     * @param name 取得対象NameComponent
     *
     * @return 指定したネームコンポーネントの文字列長さ
     *
     * @else
     * @brief Get string length of the name component's string representation
     *
     * Get string length of the name component's string representation.
     * In string representation, if NameComponent consists of 
     * {Nc[0],Nc[1],Nc[2]･･･}, the format of 
     * Nc[0]id.Nc[0].kind/Nc[1]id.Nc[1].kind/Nc[2].id/Nc[2].kind･･･
     * will be got.
     *
     * @param name The getting target NameComponent
     *
     * @return The string length value of specified component
     *
     * @endif
     */
    CORBA::ULong getNameLength(const CosNaming::Name& name);
    
    /*!
     * @if jp
     * @brief 文字列の分割
     *
     * 文字列を指定したデリミタで分割する。
     *
     * @param input 分割対象文字列
     * @param delimiter 分割用デリミタ
     * @param results 分割結果
     *
     * @return 分割した文字列の要素数
     *
     * @else
     * @brief Split of string
     *
     * Split string with specified delimiter.
     *
     * @param input The split target string
     * @param delimiter The delimiter for split
     * @param results Split result
     *
     * @return The number of split string elements
     *
     * @endif
     */
    unsigned int split(const std::string& input,
		       const std::string& delimiter,
		       std::vector<std::string>& results);
    
    /*!
     * @if jp
     * @brief ORB
     * @else
     * @brief ORB
     * @endif
     */
    CORBA::ORB_var m_varORB;
    
    /*!
     * @if jp
     * @brief ネームサーバ名称
     * @else
     * @brief Name of the name server
     * @endif
     */
    std::string m_nameServer;
    /*!
     * @if jp
     * @brief 指定したネームサーバのルートコンテキスト
     * @else
     * @brief The root context of specified name server
     * @endif
     */
    CosNaming::NamingContextExt_var m_rootContext;
    
  private:
    CORBA::ULong m_blLength;
    
  }; // class CorbaNaming
}; // namespace RTC

#ifdef WIN32
#pragma warning( default : 4290 )
#endif

#endif // end of RTC_CORBANAMING_H
