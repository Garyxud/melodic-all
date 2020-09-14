// -*- C++ -*-
/*!
 * @file 
 * @brief Properties test class
 * @date $Date: 2006-10-26 01:34:57 $
 * @author Shinji Kurihara
 * $Id$
 *
 */

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include "../ModuleManager.h"
#include "../Properties.h"

using namespace std;

// ModuleManagerTests をテストする [3]
class ModuleManagerTests
  : public CppUnit::TestFixture
{
  CPPUNIT_TEST_SUITE(ModuleManagerTests);
  CPPUNIT_TEST(test_load);
  CPPUNIT_TEST(test_unload);
  CPPUNIT_TEST(test_unloadAll);
  CPPUNIT_TEST(test_symbol);
  CPPUNIT_TEST(test_setLoadpath);
  CPPUNIT_TEST(test_getLoadPath);
  CPPUNIT_TEST(test_addLoadpath);
  CPPUNIT_TEST(test_getLoadedModules);
  CPPUNIT_TEST(test_getLoadableModules);
  CPPUNIT_TEST(test_allowAbsolutePath);
  CPPUNIT_TEST(test_disallowAbsolutePath);
  CPPUNIT_TEST(test_allowModuleDownload);
  CPPUNIT_TEST(test_disallowModuleDownload);
  CPPUNIT_TEST(test_findFile);
  CPPUNIT_TEST(test_fileExist);
  CPPUNIT_TEST(test_getInitFuncName);
  CPPUNIT_TEST_SUITE_END();

private:

  RTC::ModuleManager* m_pModMgr;

public:
  
  /*
   * コンストラクタ/デストラクタ [7]
   */
  ModuleManagerTests()
  {
    
  }
  
  ~ModuleManagerTests()
  {
  }
  

  /*
   * 初期化/後始末 [8]
   */
  virtual void setUp()
  {

    const char* default_properties[] = {
      "manager.modules.config_ext", "so",
      "manager.modules.config_path", "/etc/rtc",
      "manager.modules.detect_loadable", "Yes",
      "manager.modules.load_path", "/usr/lib, /usr/local/lib, /usr/local/lib/rtc",
      "manager.modules.init_func_suffix", "Init",
      "manager.modules.init_func_prefix", "",
      "manager.modules.abs_path_allowed", "Yes",
      "manager.modules.download_allowed", "Yes",
      "manager.modules.download_dir", "/tmp/rtc",
      "manager.modules.download_cleanup", "Yes",
      "manager.modules.preload", "",
      ""
    };
    
    RTC::Properties prop(default_properties);
    m_pModMgr = new RTC::ModuleManager(prop);
    m_pModMgr->load("libRTC.so");
    m_pModMgr->load("libm.so");
  }
  

  virtual void tearDown()
  { 
    m_pModMgr->unloadAll();
    delete m_pModMgr;
  }


  /* tests for string load(const string& file_name) */
  void test_load() {
    string libname;

    try {

      // Success case
      // ファイル名だけ与える。
      libname = m_pModMgr->load("libRTC.so");
      CPPUNIT_ASSERT(libname == "/usr/lib/libRTC.so");

      libname = m_pModMgr->load("libRTC.so");
      CPPUNIT_ASSERT(libname == "/usr/lib/libRTC.so");

      libname = m_pModMgr->load("libRTC.so");
      CPPUNIT_ASSERT(libname == "/usr/lib/libRTC.so");

      // ファイル名を絶対パスで与える。
      libname = m_pModMgr->load("/usr/lib/libRTC.so");
      CPPUNIT_ASSERT(libname == "/usr/lib/libRTC.so");

      // ディレクトリの区切り文字に"//"や"../"がある場合。 -> OK.
      libname = m_pModMgr->load("/usr//users/kurihara/../kurihara/Components/test/test.so");
      CPPUNIT_ASSERT(libname == "/usr//users/kurihara/../kurihara/Components/test/test.so");



      // Failure case
      // 
      // ファイル名だけ与える。 -> Error DLL open failed.
      //      libname = m_pModMgr->load("libm.a");
      //      CPPUNIT_ASSERT(libname == "/usr/lib/libm.a");

      // ディレクトリ名を与える。 -> Error DLL open failed.
      //      libname = m_pModMgr->load("OpenRTM");
      //      CPPUNIT_ASSERT(libname == "/usr/lib/OpenRTM");

      // 実行ファイルを与える。  -> Error Invalid file name.
      //      libname = m_pModMgr->load("rtm-naming");
      //      CPPUNIT_ASSERT(libname == "/usr/bin/rtm-naming");
      
      // 存在しないファイル名を与える。-> Error Invalid file name.
      //      libname = m_pModMgr->load("test.test");
      //      cout << "libname: " << libname << endl;
      //    CPPUNIT_ASSERT(libname == "/usr/lib/libRTC.so");
    }
    catch (RTC::ModuleManager::Error& e)
      {
	std::cout << "Error " << e.reason << std::endl;
      }
    catch (RTC::ModuleManager::NotFound& e)
      {
	std::cout << "NotFound " << e.name << std::endl;
      }
    catch (...)
      {
	std::cout << "other exception" << std::endl;
      }

  }


  /* tests for void unload(const string& file_name) */
  void test_unload() {
    try {
      string libname;
      libname = m_pModMgr->load("libRTC.so");
      CPPUNIT_ASSERT(libname == "/usr/lib/libRTC.so");

      // Success case
      m_pModMgr->unload("/usr/lib/libRTC.so");
      m_pModMgr->unload("/usr/lib/libm.so");



      // Failure case
      // ファイル名だけを与える。 -> NotFound. 絶対パスで与える必要がある。
      //      m_pModMgr->unload("libRTC.so");

      // loadしていないファイルをunloadする。 -> NotFound.
      //      m_pModMgr->unload("usr/users/kurihara/Components/test/test.so");

      // 一度unloadしたファイルをunloadする。 ->  NotFound.
      //      m_pModMgr->unload("/usr/lib/libRTC.so");
      
    }
    catch (RTC::ModuleManager::Error& e)
      {
	std::cout << "Error " << e.reason << std::endl;
      }
    catch (RTC::ModuleManager::NotFound& e)
      {
	std::cout << "NotFound " << e.name << std::endl;
      }
    catch (...)
      {
	std::cout << "other exception" << std::endl;
      }
  }


  /* tests for void unloadAll() */
  void test_unloadAll() {
    try {
      m_pModMgr->unloadAll();
      m_pModMgr->unloadAll();
      m_pModMgr->unloadAll();
    }
    catch (RTC::ModuleManager::Error& e)
      {
	std::cout << "Error " << e.reason << std::endl;
      }
    catch (RTC::ModuleManager::NotFound& e)
      {
	std::cout << "NotFound " << e.name << std::endl;
      }
    catch (...)
      {
	std::cout << "other exception" << std::endl;
      }
  }


  /* tests for void* symbol(const string& file_name, const string& func_name) */
  void test_symbol() {

    //============ 安藤氏作成のテストプログラムより抜粋 ==============================
    string libname;

    try
      {
	libname = m_pModMgr->load("libm.so");
      }
    catch (RTC::ModuleManager::Error& e)
      {
	std::cout << "Error " << e.reason << std::endl;
      }
    catch (RTC::ModuleManager::NotFound& e)
      {
	std::cout << "NotFound " << e.name << std::endl;
      }
    catch (...)
      {
	std::cout << "other exception" << std::endl;
      }

    typedef double (*cosine)(double);
    typedef double (*sine)(double);

    cosine _cos;
    sine   _sin;

    _cos = (cosine) m_pModMgr->symbol(libname, "cos");
    //    std::cout << (*_cos)(0.0) << std::endl;
    //    std::cout << (*_cos)(3.141592653589793238462643383279/2) << std::endl;
    CPPUNIT_ASSERT_MESSAGE("load error: cos(0.0)",
			   (*_cos)(0.0) == 1.0);
    CPPUNIT_ASSERT_MESSAGE("load error: cos(pi/2)",
			   (*_cos)(3.141592653589793238462643383279/2) < 0.01);

    _sin = (sine) m_pModMgr->symbol(libname, "sin");
    //    std::cout << (*_sin)(0.0) << std::endl;
    //    std::cout << (*_sin)(3.141592653589793238462643383279/2) << std::endl;
    CPPUNIT_ASSERT_MESSAGE("load error: sin(0.0)",
			   (*_sin)(0.0) == 0.0);
    CPPUNIT_ASSERT_MESSAGE("load error: sin(pi/2)",
			   (*_sin)(3.141592653589793238462643383279/2) == 1.0);   

    //===============================================================================

    libname = m_pModMgr->load("/usr/users/kurihara/RTM/Components/test/test.so");
    typedef void (*testHello)();

    testHello _th;
    // nm --dynamic test.soで"hello"が含まれる文字列(シンボル)を探して使用した。
    _th = (testHello) m_pModMgr->symbol(libname, "_ZN4test5helloEv");
    (*_th)();
  }


  /* tests for void setLoadpath(const vector<string>& load_path) */
  void test_setLoadpath() {
    vector<string> set_loadpath, get_loadpath;
    set_loadpath.push_back("/usr");
    set_loadpath.push_back("/usr/lib");
    set_loadpath.push_back("/usr/local/lib");
    set_loadpath.push_back("/tmp");
    set_loadpath.push_back("/usr/users/aaaaaaaaaaa/bbbbbbbbbbbb/ccccccccccccc/ddddddddddddd/eeeeeeeeeeeee/ffffffffffffffff/ggggggggggggg/hhhhhhhhhhhhh/iiiiiiiiiii");

    m_pModMgr->setLoadpath(set_loadpath);
    get_loadpath = m_pModMgr->getLoadPath();

    unsigned int size = get_loadpath.size();

    for (unsigned int i = 0; i < size; i++) {
      CPPUNIT_ASSERT(set_loadpath[i] == get_loadpath[i]);
    }
  }


  /* tests for vector<string> getLoadPath() */
  void test_getLoadPath() {
    vector<string> getlist, expectation;
    expectation.push_back("/usr/lib");
    expectation.push_back("/usr/local/lib");
    expectation.push_back("/usr/local/lib/rtc");
    getlist = m_pModMgr->getLoadPath();
    
    for (unsigned int i = 0; i < getlist.size(); i++) {
      CPPUNIT_ASSERT(getlist[i] == expectation[i]);
    }
  }


  /* tests for void addLoadpath(const vector<string>& load_path) */
  void test_addLoadpath() {
    vector<string> getlist, expectation, add_path;
    expectation.push_back("/usr/lib");
    expectation.push_back("/usr/local/lib");
    expectation.push_back("/usr/local/lib/rtc");
    expectation.push_back("/tmp");
    expectation.push_back("/hoge");
    expectation.push_back("/hoge/hoge");

    add_path.push_back("/tmp");
    add_path.push_back("/hoge");
    add_path.push_back("/hoge/hoge");
    m_pModMgr->addLoadpath(add_path);

    getlist = m_pModMgr->getLoadPath();
    for (unsigned int i = 0; i < getlist.size(); i++) {
      CPPUNIT_ASSERT(getlist[i] == expectation[i]);
    }
  }


  /* tests for vector<string> getLoadedModules() */
  void test_getLoadedModules() {
    vector<string> get_modlist;

    get_modlist = m_pModMgr->getLoadedModules();
    CPPUNIT_ASSERT(get_modlist[0] == "/usr/lib/libRTC.so");
    CPPUNIT_ASSERT(get_modlist[1] == "/usr/lib/libm.so");
  }


  /* tests for vector<string> getLoadableModules() */
  void test_getLoadableModules() {
    // ModuelManager.cppで実装されていないl。

    vector<string> get_modlist;
    get_modlist = m_pModMgr->getLoadableModules();
    for (unsigned int i = 0; i < get_modlist.size(); i++) {
      cout << get_modlist[i] << endl;
    }
    
  }


  /* tests for void allowAbsolutePath() */
  void test_allowAbsolutePath() {
    string libname;
    try {

      // Success case
      m_pModMgr->allowAbsolutePath();
      libname = m_pModMgr->load("libRTC.so");
      m_pModMgr->unload(libname);

      libname = m_pModMgr->load("/usr/lib/libm.so");
      m_pModMgr->unload(libname);

      libname = m_pModMgr->load("../lib/libm.so");
      m_pModMgr->unload(libname);
      

      m_pModMgr->disallowAbsolutePath();
      libname = m_pModMgr->load("../lib/libRTC.so");
      m_pModMgr->unload(libname);

      // Failure case
      //      libname = m_pModMgr->load("/usr/lib/libRTC.so");
      //      m_pModMgr->unload(libname);
    }
    catch (RTC::ModuleManager::Error& e)
      {
	std::cout << "Error " << e.reason << std::endl;
      }
    catch (RTC::ModuleManager::NotFound& e)
      {
	std::cout << "NotFound " << e.name << std::endl;
      }
    catch (...)
      {
	std::cout << "other exception" << std::endl;
      }
  }


  /* tests for void disallowAbsolutePath() */
  void test_disallowAbsolutePath() {
    // test_allowAbsolutePath()にてテスト済み。
  }


  /* tests for void allowModuleDownload() */
  void test_allowModuleDownload() {
    // テストまだ
  }


  /* tests for void disallowModuleDownload() */
  void test_disallowModuleDownload() {
    // テストまだ
  }


  /* tests for string findFile(const string& fname, const vector<string>& load_path) */
  void test_findFile() {
    //============ 安藤氏作成のテストプログラムより抜粋 ==============================
    std::string result;
    std::vector<std::string> path;
    path.push_back("/lib");
    path.push_back("/usr/lib");
    path.push_back("/usr/local/lib");

    result = m_pModMgr->findFile("libm.so", path);
    CPPUNIT_ASSERT_MESSAGE("fileFile error: libm.so",
			   result == "/usr/lib/libm.so");

    result = m_pModMgr->findFile("libc.so", path);
    CPPUNIT_ASSERT_MESSAGE("fileFile error: libc.so",
			   result == "/usr/lib/libc.so");

    result = m_pModMgr->findFile("libACE.so", path);
    CPPUNIT_ASSERT_MESSAGE("fileFile error: libACE.so",
			   result == "/usr/lib/libACE.so");

    result = m_pModMgr->findFile("hosts", path);
    CPPUNIT_ASSERT_MESSAGE("fileFile error: hosts",
			   result == "");

    result = m_pModMgr->findFile("munyamunya", path);
    CPPUNIT_ASSERT_MESSAGE("fileFile error: munyamunya",
			   result == "");
   
    //================================================================================

    // Failure case
    //    path.clear();
    //    result = m_pModMgr->findFile("libm.so", path);
    //    CPPUNIT_ASSERT_MESSAGE("fileFile error: libm.so",
    //    			   result == "/usr/lib/libm.so");

  }


  /* tests for bool fileExist(const string& filename) */
  void test_fileExist() {
    // Success case
    CPPUNIT_ASSERT(m_pModMgr->fileExist("../../../../../../../../usr/lib/libm.so"));
    CPPUNIT_ASSERT(m_pModMgr->fileExist("/usr/lib/libm.so"));

    // Failure case
    //    CPPUNIT_ASSERT(m_pModMgr->fileExist("libm.so"));
  }


  /* tests for string getInitFuncName(const string& file_path) */
  void test_getInitFuncName() {
    std::string result;

    //============ 安藤氏作成のテストプログラムより抜粋 ==============================
    result = m_pModMgr->getInitFuncName("Manipulator");
    CPPUNIT_ASSERT_MESSAGE("getInitFuncName error: Manipulator",
			   result == "ManipulatorInit");

    result = m_pModMgr->getInitFuncName("PHANToM");
    CPPUNIT_ASSERT_MESSAGE("getInitFuncName error: PHANToM",
			   result == "PHANToMInit");
    //================================================================================
  }
};


/*
 * register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ModuleManagerTests);



int main(int argc, char* argv[])
{
    CppUnit::TextUi::TestRunner runner;

    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
   
    return runner.run();
}
