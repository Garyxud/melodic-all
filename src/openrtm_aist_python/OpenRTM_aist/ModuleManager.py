#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ModuleManager.py
# @brief Loadable modules manager class
# @date $Date: 2007/08/24$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import string
import sys,os
import glob

import OpenRTM_aist


CONFIG_EXT    = "manager.modules.config_ext"
CONFIG_PATH   = "manager.modules.config_path"
DETECT_MOD    = "manager.modules.detect_loadable"
MOD_LOADPTH   = "manager.modules.load_path"
INITFUNC_SFX  = "manager.modules.init_func_suffix"
INITFUNC_PFX  = "manager.modules.init_func_prefix"
ALLOW_ABSPATH = "manager.modules.abs_path_allowed"
ALLOW_URL     = "manager.modules.download_allowed"
MOD_DWNDIR    = "manager.modules.download_dir"
MOD_DELMOD    = "manager.modules.download_cleanup"
MOD_PRELOAD   = "manager.modules.preload"



##
# @if jp
#
# @brief モジュールマネージャクラス
# @class ModuleManager
#
# モジュールのロード、アンロードなどを管理するクラス
#
# @since 0.4.0
#
# @else
#
# @biref ModuleManager class
#
# @endif
class ModuleManager:
  """
  """



  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ。
  # 設定された Property オブジェクト内の情報を基に初期化を実行する。
  #
  # @param self
  # @param prop 初期化用プロパティ
  #
  # @else
  #
  # @brief constructor
  #
  # @endif
  def __init__(self, prop):
    self._properties = prop

    self._configPath = prop.getProperty(CONFIG_PATH).split(",")
    for i in range(len(self._configPath)):
      tmp = [self._configPath[i]]
      OpenRTM_aist.eraseHeadBlank(tmp)
      self._configPath[i] = tmp[0]
    self._loadPath = prop.getProperty(MOD_LOADPTH,"./").split(",")
    for i in range(len(self._loadPath)):
      tmp = [self._loadPath[i]]
      OpenRTM_aist.eraseHeadBlank(tmp)
      self._loadPath[i] = tmp[0]

    self._absoluteAllowed = OpenRTM_aist.toBool(prop.getProperty(ALLOW_ABSPATH),
                                                "yes", "no", False)

    self._downloadAllowed = OpenRTM_aist.toBool(prop.getProperty(ALLOW_URL),
                                                "yes", "no", False)

    self._initFuncSuffix = prop.getProperty(INITFUNC_SFX)
    self._initFuncPrefix = prop.getProperty(INITFUNC_PFX)
    self._modules = OpenRTM_aist.ObjectManager(self.DLLPred)
    self._rtcout = None
    self._mgr = OpenRTM_aist.Manager.instance()

  ##
  # @if jp
  #
  # @brief デストラクタ(未実装)
  #
  # @param self
  #
  # @else
  #
  # @brief destructor
  #
  # @endif
  def __del__(self):
    self.unloadAll()


  ##
  # @if jp
  # @class Error
  # @brief ファイル・オープン失敗例外処理用内部クラス
  # @else
  #
  # @endif
  class Error:
    def __init__(self, reason_):
      self.reason = reason_



  ##
  # @if jp
  # @class NotFound
  # @brief 未実装部，指定モジュール不明例外処理用内部クラス
  # @else
  #
  # @endif
  class NotFound:
    def __init__(self, name_):
      self.name = name_



  ##
  # @if jp
  # @class FileNotFound
  # @brief 指定ファイル不明例外処理用内部クラス
  # @else
  #
  # @endif
  class FileNotFound(NotFound):
    def __init__(self, name_):
      ModuleManager.NotFound.__init__(self, name_)



  ##
  # @if jp
  # @class ModuleNotFound
  # @brief 指定モジュール不明例外処理用内部クラス
  # @else
  #
  # @endif
  class ModuleNotFound(NotFound):
    def __init__(self, name_):
      ModuleManager.NotFound.__init__(self, name_)



  ##
  # @if jp
  # @class SymbolNotFound
  # @brief 指定シンボル不明例外処理用内部クラス
  # @else
  #
  # @endif
  class SymbolNotFound(NotFound):
    def __init__(self, name_):
      ModuleManager.NotFound.__init__(self, name_)



  ##
  # @if jp
  # @class NotAllowedOperation
  # @brief 指定操作禁止時例外処理用内部クラス
  # @else
  #
  # @endif
  class NotAllowedOperation(Error):
    def __init__(self, reason_):
      ModuleManager.Error.__init__(self, reason_)
      ModuleManager.Error.__init__(self, reason_)



  ##
  # @if jp
  # @class InvalidArguments
  # @brief 指定引数不正時例外処理用内部クラス
  # @else
  #
  # @endif
  class InvalidArguments(Error):
    def __init__(self, reason_):
      ModuleManager.Error.__init__(self, reason_)



  ##
  # @if jp
  # @class InvalidOperation
  # @brief 指定操作不正時例外処理用内部クラス
  # @else
  #
  # @endif
  class InvalidOperation(Error):
    def __init__(self, reason_):
      ModuleManager.Error.__init__(self, reason_)



  ##
  # @if jp
  #
  # @brief モジュールのロード、初期化
  #
  # 指定したファイルを共有ライブラリとしてロードするとともに、
  # 指定した初期化用オペレーションを実行する。
  # 
  # @param self
  # @param file_name ロード対象モジュール名 (.pyを除いたファイル名)
  # @param init_func 初期化処理用オペレーション(デフォルト値:None)
  #
  # @return 指定したロード対象モジュール名
  #
  # @else
  #
  # @brief Load module
  #
  #
  # @endif
  # std::string ModuleManager::load(const std::string& file_name,
  #                                 const std::string& init_func)
  def load(self, file_name, init_func=None):
    if not self._rtcout:
      self._rtcout = self._mgr.getLogbuf("ModuleManager")

    self._rtcout.RTC_TRACE("load(fname = %s)", file_name)
    if file_name == "":
      raise ModuleManager.InvalidArguments, "Invalid file name."

    if OpenRTM_aist.isURL(file_name):
      if not self._downloadAllowed:
        raise ModuleManager.NotAllowedOperation, "Downloading module is not allowed."
      else:
        raise ModuleManager.NotFound, "Not implemented."

    import_name = os.path.split(file_name)[-1]
    pathChanged=False
    file_path = None
    if OpenRTM_aist.isAbsolutePath(file_name):
      if not self._absoluteAllowed:
        raise ModuleManager.NotAllowedOperation, "Absolute path is not allowed"
      else:
        splitted_name = os.path.split(file_name)
        save_path = sys.path[:]
        sys.path.append(splitted_name[0])
        pathChanged = True
        import_name = splitted_name[-1]
        file_path = file_name

    else:
      file_path = self.findFile(file_name, self._loadPath)
      if not file_path:
        raise ModuleManager.InvalidArguments, "Invalid file name."

    if not self.fileExist(file_path):
      raise ModuleManager.FileNotFound, file_name

    if not pathChanged:
      splitted_name = os.path.split(file_path)
      sys.path.append(splitted_name[0])

    ext_pos = import_name.find(".py")
    if ext_pos > 0:
      import_name = import_name[:ext_pos]
    mo = __import__(str(import_name))

    if pathChanged:
      sys.path = save_path

    dll = self.DLLEntity(mo,OpenRTM_aist.Properties())
    dll.properties.setProperty("file_path",file_path)
    self._modules.registerObject(dll)


    if init_func is None:
      return file_name

    self.symbol(file_path,init_func)(self._mgr)

    return file_name


  ##
  # @if jp
  # @brief モジュールのアンロード
  #
  # 指定したロード済みモジュールをクローズし、アンロードする。
  #
  # @param self
  # @param file_name アンロード対象モジュール名
  #
  # @else
  # @brief Unload module
  # @endif
  def unload(self, file_name):
    dll = self._modules.find(file_name)
    if not dll:
      raise ModuleManager.NotFound, file_name

    self._modules.unregisterObject(file_name)
    return


  ##
  # @if jp
  # @brief 全モジュールのアンロード
  #
  # 全てのロード済みモジュールをアンロードする。
  #
  # @param self
  #
  # @else
  # @brief Unload all modules
  # @endif
  def unloadAll(self):
    dlls = self._modules.getObjects()
    
    for dll in dlls:
      ident = dll.properties.getProperty("file_path")
      self._modules.unregisterObject(ident)
    return


  ##
  # @if jp
  # @brief モジュールのシンボルの参照
  #
  # モジュールのシンボルを取得する
  #
  # @param self
  # @param file_name 取得対象ファイル名
  # @param func_name 取得対象関数名
  #
  # @else
  # @brief Look up a named symbol in the module
  # @endif
  def symbol(self, file_name, func_name):
    dll = self._modules.find(file_name)
    if not dll:
      raise ModuleManager.ModuleNotFound, file_name

    func = getattr(dll.dll,func_name,None)

    if not func:
      raise ModuleManager.SymbolNotFound, func_name
    
    return func


  ##
  # @if jp
  # @brief モジュールロードパスを指定する
  # 
  # モジュールロード時に対象モジュールを検索するパスを指定する。
  #
  # @param self
  # @param load_path_list モジュール検索対象パスリスト
  #
  # @else
  # @brief Set default module load path
  # @endif
  def setLoadpath(self, load_path_list):
    self._loadPath = load_path_list
    return


  ##
  # @if jp
  # @brief モジュールロードパスを取得する
  # 
  # 設定されているモジュールを検索対象パスリストを取得する。
  #
  # @param self
  # 
  # @return load_path モジュール検索対象パスリスト
  #
  # @else
  # @brief Get default module load path
  # @endif
  def getLoadPath(self):
    return self._loadPath


  ##
  # @if jp
  # @brief モジュールロードパスを追加する
  # 
  # 指定されたパスリストを検索対象パスリストに追加する。
  #
  # @param self
  # @param load_path 追加モジュール検索対象パスリスト
  #
  # @else
  # @brief Add module load path
  # @endif
  def addLoadpath(self, load_path):
    for path in load_path:
      self._loadPath.append(path)
    return


  ##
  # @if jp
  # @brief ロード済みのモジュールリストを取得する
  #
  # 既にロード済みのモジュールリストを取得する。
  #
  # @param self
  #
  # @return ロード済みモジュールリスト
  #
  # @else
  # @brief Get loaded module names
  # @endif
  # std::vector<coil::Properties> getLoadedModules();
  def getLoadedModules(self):
    dlls = self._modules.getObjects()
    modules = []
    for dll in dlls:
      modules.append(dll.properties)

    return modules


  def __getRtcProfile(self, fname):
    # file name with full path
    fullname  = fname
    # directory name
    dirname   = os.path.dirname(fname)
    sys.path.append(dirname)
    # basename
    basename  = os.path.basename(fname)
    # classname
    classname  = basename.split(".")[0].lower()

    # loaded profile = old profiles - new profiles
    # for old
    oldp = self._mgr.getFactoryProfiles()

    # for new
    comp_spec_name = classname+"_spec"

    try:
      imp_file = __import__(basename.split(".")[0])
    except:
      return None
    comp_spec = getattr(imp_file,comp_spec_name,None)
    if not comp_spec:
      return None
    newp = OpenRTM_aist.Properties(defaults_str=comp_spec)

    profs = []
    
    exists = False
    for i in range(len(oldp)):
      if    oldp[i].getProperty("implementation_id") == newp.getProperty("implementation_id") and \
            oldp[i].getProperty("type_name") == newp.getProperty("type_name") and \
            oldp[i].getProperty("description") == newp.getProperty("description") and \
            oldp[i].getProperty("version") == newp.getProperty("version"):
        exists = True
    if not exists:
      profs.append(newp)

        
    # loaded component profile have to be one
    if len(profs) == 0:
      return OpenRTM_aist.Properties()

    if len(profs) > 1:
      return None

    return profs[0]


  ##
  # @if jp
  # @brief ロード可能モジュールリストを取得する(未実装)
  #
  # ロード可能なモジュールのリストを取得する。
  #
  # @param self
  #
  # @return ロード可能モジュールリスト
  #
  # @else
  # @brief Get loadable module names
  # @endif
  def getLoadableModules(self):
    # getting loadable module file path list.
    modules_ = []
    for path in self._loadPath:
      if path == "":
        continue

      flist = glob.glob(path + os.sep + '*.py')
      for file in flist:
        if file.find("__init__.py") == -1:
          modules_.append(file)
    
    props = []
    # getting module properties from loadable modules
    for mod_ in modules_:
      prop = self.__getRtcProfile(mod_)
      if prop:
        prop.setProperty("module_file_name",os.path.basename(mod_))
        prop.setProperty("module_file_path", mod_)
        props.append(prop)

    return props



  ##
  # @if jp
  # @brief モジュールの絶対パス指定許可
  #
  # ロード対象モジュールの絶対パス指定を許可するように設定する。
  #
  # @param self
  #
  # @else
  # @brief Allow absolute load path
  # @endif
  def allowAbsolutePath(self):
    self._absoluteAllowed = True


  ##
  # @if jp
  # @brief モジュールの絶対パス指定禁止
  #
  # ロード対象モジュールの絶対パス指定を禁止するように設定する。
  #
  # @param self
  #
  # @else
  # @brief Forbid absolute load path
  # @endif
  def disallowAbsolutePath(self):
    self._absoluteAllowed = False


  ##
  # @if jp
  # @brief モジュールのURL指定許可
  #
  # ロード対象モジュールのURL指定を許可する。
  # 本設定が許可されている場合、モジュールをダウンロードしてロードすることが
  # 許可される。
  #
  # @param self
  #
  # @else
  # @brief Allow module download
  # @endif
  def allowModuleDownload(self):
    self._downloadAllowed = True


  ##
  # @if jp
  # @brief モジュールのURL指定禁止
  #
  # ロード対象モジュールのURL指定を禁止する。
  #
  # @param self
  #
  # @else
  # @brief Forbid module download
  # @endif
  def disallowModuleDownload(self):
    self._downloadAllowed = False


  ##
  # @if jp
  # @brief LoadPath からのファイルの検索
  # 
  # 指定されたパス内に、指定されたファイルが存在するか確認する。
  #
  # @param self
  # @param fname 検索対象ファイル名
  # @param load_path 検索先パスリスト
  #
  # @return 検索されたファイル名
  #
  # @else
  # @brief Search file from load path
  # @endif
  def findFile(self, fname, load_path):
    file_name = fname
    for path in load_path:
      if fname.find(".py") == -1:
        f = str(path) + os.sep + str(file_name)+".py"
      else:
        f = str(path)+ os.sep + str(file_name)
      if self.fileExist(f):
        return f
    return ""


  ##
  # @if jp
  # @brief ファイルが存在するかどうかのチェック
  #
  # 指定されたファイルが存在するか確認する。
  #
  # @param self
  # @param filename 存在確認対象ファイル名
  #
  # @return ファイル存在確認結果(ファイルあり:true，なし:false)
  #
  # @else
  # @brief Check file existance
  # @endif
  def fileExist(self, filename):
    fname = filename
    if fname.find(".py") == -1:
      fname = str(filename)+".py"

    if os.path.isfile(fname):
      return True

    return False



  ##
  # @if jp
  # @brief 初期化関数シンボルを生成する
  #
  # 初期化関数の名称を組み立てる。
  #
  # @param self
  # @param file_path 初期化対象モジュール名称
  #
  # @return 初期化関数名称組み立て結果
  #
  # @else
  # @brief Create initialize function symbol
  # @endif
  def getInitFuncName(self, file_path):
    base_name = os.path.basename(file_path)
    return str(self._initFuncPrefix)+str(base_name)+str(self._initFuncSuffix)



  ##
  # @if jp
  # @class DLL
  # @brief モジュール保持用内部クラス
  # @else
  #
  # @endif
  class DLL:
    def __init__(self, dll):
      self.dll = dll
      return


  class DLLEntity:
    def __init__(self,dll,prop):
      self.dll = dll
      self.properties = prop


  class DLLPred:
    def __init__(self, name=None, factory=None):
      self._filepath = name or factory

    def __call__(self, dll):
      return self._filepath == dll.properties.getProperty("file_path")
