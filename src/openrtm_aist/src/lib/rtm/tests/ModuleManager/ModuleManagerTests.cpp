// -*- C++ -*-
/*!
 * @file   ModuleManagerTests.cpp
 * @brief  ModuleManager test class
 * @date   $Date: 2008/05/01 03:42:41 $
 * @author Shinji Kurihara
 *         Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006
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

/*
 * $Log: ModuleManagerTests.cpp,v $
 * Revision 1.2  2008/05/01 03:42:41  arafune
 * Modified some tests.
 *
 * Revision 1.1  2007/12/20 07:50:17  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2006/11/27 08:33:02  n-ando
 * TestSuites are devided into each directory.
 *
 * Revision 1.2  2006/10/26 01:34:57  kurihara
 * test program of class ModuelManager.
 *
 * Revision 1.1  2006/09/20 08:52:19  n-ando
 * The first commit.
 *
 */

#ifndef ModuleManager_cpp
#define ModuleManager_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <fstream>
#include <rtm/ModuleManager.h>
#include <coil/Properties.h>

/*!
 * @class ModuleManagerTests class
 * @brief ModuleManager test
 */
namespace ModuleManager
{
  class ModuleManagerTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ModuleManagerTests);
    CPPUNIT_TEST(test_load);
    CPPUNIT_TEST(test_load_inexistent_on_load_path);
    CPPUNIT_TEST(test_unload);
    CPPUNIT_TEST(test_unloadAll);
    CPPUNIT_TEST(test_symbol);
    CPPUNIT_TEST(test_setLoadpath_and_getLoadPath);
    CPPUNIT_TEST(test_addLoadpath);
    //CPPUNIT_TEST(test_getLoadedModules);
    CPPUNIT_TEST(test_getLoadableModules);
    CPPUNIT_TEST(test_allowAbsolutePath_and_disallowAbsolutePath);
    //CPPUNIT_TEST(test_allowModuleDownload);
    //CPPUNIT_TEST(test_disallowModuleDownload);
    CPPUNIT_TEST(test_findFile);
    CPPUNIT_TEST(test_fileExist);
    CPPUNIT_TEST(test_getInitFuncName);
    CPPUNIT_TEST_SUITE_END();
		
  private:
    RTC::ModuleManager* m_pModMgr;
		
  private:
    bool isFound(const std::vector<coil::Properties>& path, const std::string& mod)
    {
      for (int i(0), len(path.size()); i < len; ++i)
        {
          if (mod == path[i]["file_path"]) return true;
        }
      return false;
    }
	
  public:
    /*!
     * @brief Constructor
     */
    ModuleManagerTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~ModuleManagerTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      const char* default_properties[] =
	{
	  "manager.modules.config_ext", "so",
	  "manager.modules.config_path", "/etc/rtc",
	  "manager.modules.detect_loadable", "Yes",
	  "manager.modules.load_path",
	  "../../.libs, .libs, /usr/lib, /usr/local/lib, /usr/local/lib/rtc",
	  "manager.modules.init_func_suffix", "Init",
	  "manager.modules.init_func_prefix", "",
	  "manager.modules.abs_path_allowed", "Yes",
	  "manager.modules.download_allowed", "Yes",
	  "manager.modules.download_dir", "/tmp/rtc",
	  "manager.modules.download_cleanup", "Yes",
	  "manager.modules.preload", "",
	  ""
	};
			
      coil::Properties prop(default_properties);
      m_pModMgr = new RTC::ModuleManager(prop);
			
      // ロードパス上にlibRTC.soが存在することを確認しておく
      m_pModMgr->load("libRTC.so");
			
      // ロードパス上にlibm.soが存在することを確認しておく
      m_pModMgr->load("libm.so");
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
      m_pModMgr->unloadAll();
      delete m_pModMgr;
    }
		
    /* tests for string load(const string& file_name) */
    /*!
     * @brief load()メソッドのテスト
     * 
     * - ファイル名だけ指定した場合に、正常にロードできるか？
     * - ファイル名を絶対パスで指定した場合に、正常にロードできるか？
     * - パス内に、「//」や「../」が含まれる場合に、正常にロードできるか？
     * - ロードパス上にファイルは存在するが、拡張子が適用外である場合に、意図どおりロード失敗するか？
     * - 存在しないファイルを指定した場合に、意図どおりにロード失敗するか？
     */
    void test_load()
    {
      try
	{
	  // Success case
	  // ファイル名だけ指定した場合に、ロードできるか？
	  //std::string str = m_pModMgr->load("libRTC.so");
	  //std::cout << "\nstr=(" << str << std::endl;
	  // インストール環境によって、ファイルパスが変化するのでコメントにする
//	  CPPUNIT_ASSERT_EQUAL(
//			       std::string("/usr/local/lib/libRTC.so"),
//			       std::string("../../.libs/libRTC.so"),
//			       std::string("/usr/lib/libRTC.so"),
//			       m_pModMgr->load("libRTC.so"));
				
/*
	  // ファイル名を絶対パスで指定した場合に、正常にロードできるか？
	  CPPUNIT_ASSERT_EQUAL(
//			       std::string("/usr/local/lib/libRTC.so"),
			       std::string("../../.libs/libRTC.so"),
			       m_pModMgr->load("/usr/local/lib/libRTC.so"));
				
	  // ディレクトリの区切り文字に"//"や"../"がある場合
	  CPPUNIT_ASSERT_EQUAL(
//			       std::string("/usr/local//lib/../lib/libRTC.so"),
//			       m_pModMgr->load("/usr/local//lib/../lib/libRTC.so"));
			       std::string("../..//.libs/../libs/libRTC.so"),
			       m_pModMgr->load("../..//.libs/../libs/libRTC.so"));
*/
	}
      catch (RTC::ModuleManager::Error& e)
	{
	  CPPUNIT_FAIL("Error" + e.reason);
	}
      catch (RTC::ModuleManager::NotFound& e)
	{
	  CPPUNIT_FAIL("NotFound" + e.name);
	}
      catch (RTC::ModuleManager::NotAllowedOperation& e)
	{
	  CPPUNIT_FAIL("NotAllowedOperation");
	}
      catch (RTC::ModuleManager::InvalidArguments& e)
	{
	  CPPUNIT_FAIL("InvalidArguments");
	}
      catch (RTC::ModuleManager::FileNotFound& e)
	{
	  CPPUNIT_FAIL("FileNotFound" + e.name);
	}
      catch (...)
	{
	  CPPUNIT_FAIL("Other Exception");
	}
			
      // Failure case
      bool notThrown;

      // ロードパス上にファイルは存在するが、拡張子が適用外である場合に、意図どおりロード失敗するか？
      notThrown = false;
      try
	{
	  m_pModMgr->load("libm.a");
	  notThrown = true;
	}
      catch (...) {}
      if (notThrown) CPPUNIT_FAIL("Exception not thrown.");

      // 存在しないファイルの場合
      notThrown = false;
      try
	{
	  m_pModMgr->load("inexist-file");
	  notThrown = true;
	}
      catch (...) {}
      if (notThrown) CPPUNIT_FAIL("Exception not thrown.");
    }
		
    /*!
     * @brief load()メソッドのテスト
     * 
     * - ロードパス上に存在しないモジュールのロードを試みた場合、意図どおりロード失敗するか？
     */
    void test_load_inexistent_on_load_path()
    {

      std::vector<std::string> loadPath = m_pModMgr->getLoadPath();
			
      // setLoadPath()を利用して、ロードパスをクリアする
      std::vector<std::string> emptyPath;
      m_pModMgr->setLoadpath(emptyPath);
			
      // ロードパス上に存在しないモジュールのロードを試みた場合、意図どおりロード失敗するか？
      // （ロードパスをクリアした状態なので、モジュールロードに失敗するはず）
      bool notThrown = false;
      try
	{
	  m_pModMgr->load("libm.so");
	  notThrown = true;
	}
      catch (...) {}
      if (notThrown) CPPUNIT_FAIL("Exception not thrown.");
			
      // 元のロードパスを設定しなおす
      m_pModMgr->setLoadpath(loadPath);
			
      // モジュールのロードに成功するはず
      std::string modName = m_pModMgr->load("libm.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName));
    }
		
    /*!
     * @brief unload()メソッドのテスト
     * 
     * - ロードしておいたモジュールを正しくアンロードできるか？
     * - アンロードしていないモジュールは、なおアンロードされずに残っているか？
     * - 絶対パスを指定せず、ファイル名だけ指定した場合に、意図どおりにアンロード失敗するか？
     * - ロードしていないモジュールのアンロードを試みた場合、意図どおりに失敗するか？
     * - アンロード済みのモジュールを、さらにアンロードしようと試みた場合、意図どおりに失敗するか？
     */
    void test_unload()
    {

      // モジュールをロードしておく
      std::string modName1 = m_pModMgr->load("libRTC.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName1));
			
      std::string modName2 = m_pModMgr->load("libm.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName2));
			
      // Success case
      // ロードしておいたモジュールを正しくアンロードできるか？
      m_pModMgr->unload(modName1);
      CPPUNIT_ASSERT(! isFound(m_pModMgr->getLoadedModules(), modName1));
			
      // アンロードしていないモジュールは、なおアンロードされずに残っているか？
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName2));
			
			
      // Failure case
      // 絶対パスを指定せず、ファイル名だけ指定した場合に、意図どおりにアンロード失敗するか？
      try
	{
	  m_pModMgr->unload("libm.so");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (RTC::ModuleManager::NotFound expected) {}
			
      // ロードしていないモジュールのアンロードを試みた場合、意図どおりに失敗するか？
      try
	{
	  m_pModMgr->unload("non-loaded-module.so");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (RTC::ModuleManager::NotFound expected) {}
			
      // アンロード済みのモジュールを、さらにアンロードしようと試みた場合、意図どおりに失敗するか？
      try
	{
	  m_pModMgr->unload(modName1);
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (RTC::ModuleManager::NotFound expected) {}
    }
		
    /*!
     * @brief unloadAll()メソッドのテスト
     * 
     * - ロード済みのモジュールがすべてアンロードされるか？
     */
    void test_unloadAll()
    {
      // モジュールをロードしておく
      std::string modName1 = m_pModMgr->load("libRTC.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName1));
			
      std::string modName2 = m_pModMgr->load("libm.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName2));
			
      // unloadAll()によって、ロード済みのモジュールがすべてアンロードされるか？
      m_pModMgr->unloadAll();
      CPPUNIT_ASSERT(! isFound(m_pModMgr->getLoadedModules(), modName1));
      CPPUNIT_ASSERT(! isFound(m_pModMgr->getLoadedModules(), modName2));
    }
		
    /*!
     * @brief symbol()メソッドのテスト
     * 
     * - モジュールが持つシンボル（関数ポインタ）を正常に取得できるか？
     * - 取得したシンボルに対する呼出を正常に行えるか？
     */
    void test_symbol()
    {
      typedef double (*FUNCTION_COS)(double);
      typedef double (*FUNCTION_SIN)(double);

      // モジュールをロードしておく
      std::string modName = m_pModMgr->load("libm.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName));
			
      // "cos"関数へのシンボルを取得する
      FUNCTION_COS func_cos = (FUNCTION_COS) m_pModMgr->symbol(modName, "cos");
      CPPUNIT_ASSERT(func_cos != NULL);
			
      // "sin"関数へのシンボルを取得する
      FUNCTION_SIN func_sin = (FUNCTION_SIN) m_pModMgr->symbol(modName, "sin");
      CPPUNIT_ASSERT(func_sin != NULL);
			
      // 取得したシンボルへの呼出を正常に行えるか？
      CPPUNIT_ASSERT_EQUAL(1.0, (*func_cos)(0.0));
      CPPUNIT_ASSERT_EQUAL(0.0, (*func_sin)(0.0));
    }
		
    /*!
     * @brief setLoadpath()とgetLoadPath()のテスト
     * 
     * - ロードパスを正しく設定できるか？
     * - ロードパスを正しく取得できるか？
     */
    void test_setLoadpath_and_getLoadPath()
    {

      std::vector<std::string> loadPath;
      loadPath.push_back("/usr");
      loadPath.push_back("/usr/lib");
			
      // setLoadpath()で設定したパスを、getLoadPath()で取得できるか？
      m_pModMgr->setLoadpath(loadPath);
      std::vector<std::string> loadPathRet = m_pModMgr->getLoadPath();
      CPPUNIT_ASSERT_EQUAL(2, (int) loadPathRet.size());
      CPPUNIT_ASSERT_EQUAL(std::string("/usr"), loadPathRet[0]);
      CPPUNIT_ASSERT_EQUAL(std::string("/usr/lib"), loadPathRet[1]);

    }
		
    /*!
     * @brief addLoadpath()メソッドのテスト
     * 
     * - 正しくロードパスを追加できるか？
     */
    void test_addLoadpath()
    {
      std::vector<std::string> loadPath1;
      loadPath1.push_back("/usr");
      loadPath1.push_back("/usr/lib");
      std::sort(loadPath1.begin(), loadPath1.end());

      std::vector<std::string> loadPath2;
      loadPath2.push_back("/usr/local/lib");
      loadPath2.push_back("/usr/local/lib/rtc");
      std::sort(loadPath2.begin(), loadPath2.end());
			
      std::vector<std::string> expectedPath;
      expectedPath.push_back("/usr");
      expectedPath.push_back("/usr/lib");
      expectedPath.push_back("/usr/local/lib");
      expectedPath.push_back("/usr/local/lib/rtc");
      std::sort(expectedPath.begin(), expectedPath.end());
			
      // まず初期状態のロードパスを設定しておく
      std::vector<std::string> loadPathRet;
      m_pModMgr->setLoadpath(loadPath1);
      loadPathRet = m_pModMgr->getLoadPath();
      CPPUNIT_ASSERT_EQUAL(2, (int) loadPathRet.size());
      for (int i = 0; i < 2; ++i) {
	CPPUNIT_ASSERT_EQUAL(loadPath1[i], loadPathRet[i]);
      }
			
      // 正しくロードパスを追加できるか？
      m_pModMgr->addLoadpath(loadPath2);
      loadPathRet = m_pModMgr->getLoadPath();
      CPPUNIT_ASSERT_EQUAL(4, (int) loadPathRet.size());
      for (int i = 0; i < 4; ++i) {
	CPPUNIT_ASSERT_EQUAL(expectedPath[i], loadPathRet[i]);
      }
    }
		
    /*!
     * @brief getLoadedModules()メソッドのテスト
     */
    void test_getLoadedModules()
    {
      // 他テスト内で使用されており、それらで兼ねるものとする
    }
		
    /*!
     * @brief getLoadableModules()メソッドのテスト
     */
    void test_getLoadableModules()
    {
      std::vector<coil::Properties> props = m_pModMgr->getLoadableModules();
      int len = props.size();
      CPPUNIT_ASSERT(len > 0);

      bool bret = false;
      for(int i(0); i<len; ++i)
      {
        if(props[i].getProperty("module_file_name") == "DummyModule1.so")
        {
          bret = true;
          break;
        }
      }
      CPPUNIT_ASSERT(bret);
    }
		
    /*!
     * @brief allowAbsolutePath()メソッドとdisallowAbsolutePath()メソッドのテスト
     * 
     * - 絶対パス指定を許可した状態で、絶対パス指定でモジュールロードできるか？
     * - 絶対パス指定を禁止した状態で、絶対パス指定でモジュールロードを試みて、意図どおり失敗するか？
     */
    void test_allowAbsolutePath_and_disallowAbsolutePath()
    {
      // 絶対パス指定を許可した状態で、絶対パス指定でモジュールロードできるか？
      m_pModMgr->allowAbsolutePath();
      std::string modName = m_pModMgr->load("/usr/lib/libm.so");
      CPPUNIT_ASSERT(isFound(m_pModMgr->getLoadedModules(), modName));
			
      // 絶対パス指定を禁止した状態で、絶対パス指定でモジュールロードを試みて、意図どおり失敗するか？
      m_pModMgr->unloadAll(); // いったんアンロードしておく
      m_pModMgr->disallowAbsolutePath();
      try
	{
	  m_pModMgr->load("/usr/lib/libm.so");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (RTC::ModuleManager::NotAllowedOperation expected) {}
    }
		
    /*!
     * @brief allowModuleDownload()メソッドのテスト
     */
    void test_allowModuleDownload()
    {
      // ダウンロードによるモジュールロード処理が未実装のため、テストも未実装
    }
		
    /*!
     * @brief disallowModuleDownload()メソッドのテスト
     */
    void test_disallowModuleDownload()
    {
      // ダウンロードによるモジュールロード処理が未実装のため、テストも未実装
    }
		
    /*!
     * @brief findFile()メソッドのテスト
     * 
     * - 指定したパス上にファイルが存在する場合、正しく有無判定し、パスを取得できるか？
     * - 指定したパス上にファイルが存在しない場合、正しく有無判定し、空文字列を取得できるか？
     */
    void test_findFile()
    {

      std::vector<std::string> path;
      path.push_back("/usr/lib");
			
      // 指定したパス上にファイルが存在する場合、正しく有無判定し、パスを取得できるか？
      CPPUNIT_ASSERT_EQUAL(std::string("/usr/lib/libm.so"),
			   m_pModMgr->findFile("libm.so", path));
			
      // 指定したパス上にファイルが存在しない場合、正しく有無判定し、空文字列を取得できるか？
      CPPUNIT_ASSERT_EQUAL(std::string(""),
			   m_pModMgr->findFile("inexist-file.so", path));
    }
		
    /*!
     * @brief fileExist()メソッドのテスト
     * 
     * - 存在するファイルを指定した場合に、正しく判定されるか？
     * - 存在しないファイルを指定した場合に、正しく判定されるか？
     * - 存在するファイルを「//」「../」が混じったパス表記で指定した場合、正しく判定されるか？
     */
    void test_fileExist()
    {
      // 存在するファイルを指定した場合に、正しく判定されるか？
      CPPUNIT_ASSERT(m_pModMgr->fileExist("/usr/lib/libm.so"));

      // 存在しないファイルを指定した場合に、正しく判定されるか？
      CPPUNIT_ASSERT(! m_pModMgr->fileExist("/usr/lib/inexistent-file.so"));
			
      // 存在するファイルを「//」「../」が混じったパス表記で指定した場合、正しく判定されるか？
      CPPUNIT_ASSERT(m_pModMgr->fileExist("/usr//lib/../lib/libm.so"));
    }
		
    /*!
     * @brief getInitFuncName()メソッドのテスト
     * 
     * - 初期化関数名を正しく取得できるか？
     */
    void test_getInitFuncName()
    {
      // 初期化関数名を正しく取得できるか？
      CPPUNIT_ASSERT_EQUAL(std::string("ManipulatorInit"),
			   m_pModMgr->getInitFuncName("Manipulator"));
			
      CPPUNIT_ASSERT_EQUAL(std::string("PHANToMInit"),
			   m_pModMgr->getInitFuncName("PHANToM"));
    }
		
  };
}; // namespace ModuleManager

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ModuleManager::ModuleManagerTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{

  FORMAT format = TEXT_OUT;
  int target = 0;
  std::string xsl;
  std::string ns;
  std::string fname;
  std::ofstream ofs;

  int i(1);
  while (i < argc)
    {
      std::string arg(argv[i]);
      std::string next_arg;
      if (i + 1 < argc) next_arg = argv[i + 1];
      else              next_arg = "";

      if (arg == "--text") { format = TEXT_OUT; break; }
      if (arg == "--xml")
	{
	  if (next_arg == "")
	    {
	      fname = argv[0];
	      fname += ".xml";
	    }
	  else
	    {
	      fname = next_arg;
	    }
	  format = XML_OUT;
	  ofs.open(fname.c_str());
	}
      if ( arg == "--compiler"  ) { format = COMPILER_OUT; break; }
      if ( arg == "--cerr"      ) { target = 1; break; }
      if ( arg == "--xsl"       )
	{
	  if (next_arg == "") xsl = "default.xsl"; 
	  else                xsl = next_arg;
	}
      if ( arg == "--namespace" )
	{
	  if (next_arg == "")
	    {
	      std::cerr << "no namespace specified" << std::endl;
	      exit(1); 
	    }
	  else
	    {
	      xsl = next_arg;
	    }
	}
      ++i;
    }
  CppUnit::TextUi::TestRunner runner;
  if ( ns.empty() )
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
  else
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry(ns).makeTest());
  CppUnit::Outputter* outputter = 0;
  std::ostream* stream = target ? &std::cerr : &std::cout;
  switch ( format )
    {
    case TEXT_OUT :
      outputter = new CppUnit::TextOutputter(&runner.result(),*stream);
      break;
    case XML_OUT :
      std::cout << "XML_OUT" << std::endl;
      outputter = new CppUnit::XmlOutputter(&runner.result(),
					    ofs, "shift_jis");
      static_cast<CppUnit::XmlOutputter*>(outputter)->setStyleSheet(xsl);
      break;
    case COMPILER_OUT :
      outputter = new CppUnit::CompilerOutputter(&runner.result(),*stream);
      break;
    }
  runner.setOutputter(outputter);
  runner.run();
  return 0; // runner.run() ? 0 : 1;
}
#endif // MAIN
#endif // ModuleManager_cpp
