// -*- C++ -*-
/*!
 * @file ManagerConfig.h
 * @brief RTC manager configuration
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2008
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

#ifndef RTC_MANAGERCONFIG_H
#define RTC_MANAGERCONFIG_H

#include <coil/Properties.h>


namespace RTC
{
  /*!
   * @if jp
   *
   * @class ManagerConfig
   * @brief Manager configuration クラス
   *
   * Manager のコンフィギュレーションを行う、コマンドライン引数を受け取り、
   * (あるいは引数なしで)インスタンス化される。
   * コマンドライン引数で指定された設定ファイル、環境変数などから Manager の
   * プロパティ情報を設定する。
   *
   * 各設定の優先度は以下のとおりである。
   * <OL>
   * <LI>コマンドラインオプション "-f"
   * <LI>環境変数 "RTC_MANAGER_CONFIG"
   * <LI>デフォルト設定ファイル "./rtc.conf"
   * <LI>デフォルト設定ファイル "/etc/rtc.conf"
   * <LI>デフォルト設定ファイル "/etc/rtc/rtc.conf"
   * <LI>デフォルト設定ファイル "/usr/local/etc/rtc.conf"
   * <LI>デフォルト設定ファイル "/usr/local/etc/rtc/rtc.conf"
   * <LI>埋め込みコンフィギュレーション値
   *</OL>
   * ただし、コマンドラインオプション "-d" が指定された場合は、
   * (たとえ -f で設定ファイルを指定しても)埋め込みコンフィギュレーション値
   * が使用される。
   *
   * @since 0.4.0
   *
   * @else
   *
   * @class ManagerConfig
   * @brief Manager configuration class
   *
   * Modify Manager's configuration. 
   * This class receives the command line arguments and will be instantiated.
   * Set property information of Manager with the configuration file specified
   * by the command line argument or the environment variable etc.
   *
   * The priorities of each configuration are as follows:
   * <OL>
   * <LI>Command option "-f"
   * <LI>Environment variable "RTC_MANAGER_CONFIG"
   * <LI>Default configuration file "./rtc.conf"
   * <LI>Default configuration file "/etc/rtc.conf"
   * <LI>Default configuration file "/etc/rtc/rtc.conf"
   * <LI>Default configuration file "/usr/local/etc/rtc.conf"
   * <LI>Default configuration file "/usr/local/etc/rtc/rtc.conf"
   * <LI>Embedded configuration value
   *</OL>
   * If the command option "-d" is specified (even if specify configuration file
   * by "-f" option), the embedded configuration values will be used.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class ManagerConfig
  {
  public:
    // The list of default configuration file path.
    /*!
     * @if jp
     * @brief Manager コンフィギュレーションのデフォルト・ファイル・パス
     * @else
     * @brief The default configuration file path for manager
     * @endif
     */
    static const char* config_file_path[];
    
    // Environment value to specify configuration file
    /*!
     * @if jp
     * @brief デフォルト・コンフィギュレーションのファイル・パスを識別する
     *        環境変数
     * @else
     * @brief The environment variable to distinguish the default configuration
     *        file path
     * @endif
     */
    static const char* config_file_env;
    
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ。(何もしない)
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor (Do nothing)
     *
     * @endif
     */
    ManagerConfig();
    
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * 与えられた引数によりコンフィギュレーション情報の初期化を行う。
     *
     * @param argc コマンドライン引数の数
     * @param argv コマンドライン引数
     *
     * @else
     *
     * @brief Constructor
     *
     * Initialize configuration information by given arguments.
     *
     * @param argc Number of command line arguments
     * @param argv The command line arguments
     *
     * @endif
     */
    ManagerConfig(int argc, char** argv);
    
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * @endif
     */
    virtual ~ManagerConfig(void);
    
    /*!
     * @if jp
     *
     * @brief 初期化
     *
     * コマンドライン引数に応じて初期化を実行する。コマンドラインオプションは
     * 以下のものが使用可能である。
     *
     *  - -f file   : コンフィギュレーションファイルを指定する。
     *  - -l module : ロードするモジュールを指定する。(未実装)
     *  - -o options: その他オプションを指定する。(未実装)
     *  - -d        : デフォルトのコンフィギュレーションを使う。(未実装)
     *
     * @param argc コマンドライン引数の数
     * @param argv コマンドライン引数
     *
     * @else
     *
     * @brief Initialization
     *
     * Initialize with command line options. The following command options
     * are available.
     *
     * - -f file   : Specify the configuration file.
     * - -l module : Specify modules to be loaded. (Not implemented)
     * - -o options: Specify other options. (Not implemented)
     * - -d        : Use default static configuration. (Not implemented)
     *
     * @param argc Number of command line arguments
     * @param argv The command line arguments
     *
     * @endif
     */
    void init(int argc, char** argv);
    
    /*!
     * @if jp
     * @brief Configuration 情報を Property に設定する
     * 
     * Manager のConfiguration 情報を指定された Property に設定する。
     *
     * @param prop Configuration 設定対象 Property
     * 
     * @else
     * @brief Specify the configuration information to the Property
     * 
     * Configure to the properties specified by Manager's configuration
     *
     * @param prop The target properties to configure
     * 
     * @endif
     */
    void configure(coil::Properties& prop);
    
    /*!
     * @if jp
     *
     * @brief コンフィギュレーションを取得する
     *
     * コンフィギュレーションを取得する。init()呼び出し前に呼ぶと、
     * 静的に定義されたデフォルトのコンフィギュレーションを返す。
     * init() 呼び出し後に呼ぶと、コマンドライン引数、環境変数等に
     * 基づいた初期化されたコンフィギュレーションを返す。
     * (未実装)
     *
     * @return Manager のコンフィギュレーション情報
     *
     * @else
     *
     * @brief Get the configuration.
     *
     * Get the configuration.
     * When this operation is called before calling init() function,
     * return the default configuration statically defined,
     * When this operation is called after calling init() function,
     * return the initialized configuration according to
     * the command line arguments, the environment variables etc.
     * (Not implemented)
     *
     * @return Manager's configuration information
     *
     * @endif
     */
    coil::Properties getConfig() const;
    
  protected:
    /*!
     * @if jp
     *
     * @brief コマンド引数をパースする
     *
     * - -f file   : コンフィギュレーションファイルを指定する。
     * - -l module : ロードするモジュールを指定する。(未実装)
     * - -o options: その他オプションを指定する。(未実装)
     * - -d        : デフォルトのコンフィギュレーションを使う。(未実装)
     *
     * @param argc コマンドライン引数の数
     * @param argv コマンドライン引数
     *
     * @else
     *
     * @brief Parse the command arguments
     *
     *  - -f file   : Specify the configuration file.
     *  - -l module : Specify modules to be loaded. (Not implemented)
     *  - -o options: Other options. (Not implemented)
     *  - -d        : Use default static configuration. (Not implemented)
     *
     * @param argc Number of command line arguments
     * @param argv The command line arguments
     *
     * @endif
     */
    void parseArgs(int argc, char** argv);
    
    /*!
     * @if jp
     *
     * @brief Configuration file の検索
     *
     * Configuration file を検索し、設定する。
     * 既に Configuration file が設定済みの場合は、ファイルの存在確認を行う。
     *
     * Configuration file の優先順位<br>
     * コマンドオプション指定＞環境変数＞デフォルトファイル＞デフォルト設定
     *
     * デフォルト強制オプション(-d): デフォルトファイルがあっても無視して
     *                               デフォルト設定を使う
     *
     * @return Configuration file 検索結果
     *
     * @else
     *
     * @brief Find the configuration file
     *
     * Find the configuration file and configure it.
     * Confirm the file existence when the configuration file has 
     * already configured.
     *
     * The priority of the configuration file<br>
     * The command option＞the environment variable＞the default file＞
     * the default configuration
     *
     * Default force option(-d): Ignore any default files and use the default 
     * configuration.
     *
     * @return Configuration file search result
     *
     * @endif
     */
    bool findConfigFile();
    
    /*!
     * @if jp
     *
     * @brief システム情報を設定する
     *
     * システム情報を取得しプロパティにセットする。設定されるキーは以下の通り。
     *  - os.name    : OS名
     *  - os.release : OSリリース名
     *  - os.version : OSバージョン名
     *  - os.arch    : OSアーキテクチャ
     *  - os.hostname: ホスト名
     *  - manager.pid        : プロセスID
     * 
     * @param prop システム情報を設定したプロパティ
     * @else
     * 
     * @brief Set system information
     * 
     * Get the following system info. and set them to Manager's properties.
     *  - os.name    : OS name
     *  - os.release : OS release name
     *  - os.version : OS version
     *  - os.arch    : OS architecture
     *  - os.hostname: Hostname
     *  - manager.pid        : process ID
     *
     * @param prop Properties to set system information
     *
     * @endif
     */
    void setSystemInformation(coil::Properties& prop);
    
    /*!
     * @if jp
     * @brief ファイルの存在確認
     *
     * 指定されたファイルが存在するか確認する。
     *
     * @param filename 確認対象ファイル名称
     *
     * @return 対象ファイル確認結果(存在する場合にtrue)
     *
     * @else
     * @brief Check the file existence
     *
     * Confirm whether the specified file exists
     *
     * @param filename The target confirmation file
     *
     * @return file existance confirmation (True if the file exists.)
     *
     * @endif
     */
    bool fileExist(const std::string& filename);
    
    /*!
     * @if jp
     * @brief 引数から渡されるプロパティ
     * @else
     * @brief configuration properties from arguments
     * @endif
     */
    coil::Properties m_argprop;

    /*!
     * @if jp
     * @brief Manager コンフィギュレーション・ファイルのパス
     * @else
     * @brief Manager's configuration file path
     * @endif
     */
    std::string m_configFile;

    /*!
     * @if jp
     * @brief Manager マスタフラグ
     *
     * true:マスタ,false:スレーブ
     *
     * @else
     * @brief Manager master flag
     *
     * true:master,false:slave
     *
     * @endif
     */
    bool m_isMaster;
  };
}; // namespace RTC  
#endif // RTC_MANAGERCONFIG_H
