#!/usr/bin/env python
#
# @file setuptest.py
# @brief CppUnit test environment setup script
# @date $Date: 2008-02-29 04:50:54 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2006
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: setuptest.py 775 2008-07-28 16:14:45Z n-ando $
#
# [usage]
# setuptest.py [class_name]
#
# 1. make test class file and Makefile.am
# > setuptest.py
# " Hoge/Makefile.am " was generated.
# " Hoge/HogeTests.cpp " was generated.
#
# 2. add Makefile entry into the configure.ac
# > vi configure.ac
#------------------------------------------------------------
# AC_OUTPUT([Makefile
#            _test_dir_/Hoge/Makefile <- add this entry
#           ])
#------------------------------------------------------------
# 2.5 add dubsir entry to parent dir's Makefile.am
#
# 3. autoreconf and configure again
# autoreconf generate Makefile.in
# configure generate Makefile
#
# 4. buid test
# > cd  _test_dir_/Hoge
# > make
# > ./HogeTests
#
# done

import sys
import os
import yat

makefile_am = """# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for [class_name] unit test
# @date   [dollar]Date[dollar]

# @author Noriaki Ando <n-ando@aist.go.jp>
#
# [dollar]Id[dollar]

#
#------------------------------------------------------------

#
# [dollar]Log[dollar]

#


AUTOMAKE_OPTIONS = 1.9

AM_CPPFLAGS= -I.                  \\
	-I$(includedir)           \\
	-I$(top_builddir)

AM_LDFLAGS= -L.                   \\
	-L$(top_builddir)


noinst_PROGRAMS = [class_name]Tests

[class_name]Tests_SOURCES = ../TestRunner.cpp [class_name]Tests.cpp
[class_name]Tests_LDFLAGS = -L$(libdir)
[class_name]Tests_LDADD   = -lcppunit

TEST_SRC = [dollar]([class_name]Tests_SOURCES)
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./[class_name]Tests

# clean-up
clean-local:
	rm -f *.o *.Po *.gch *.la
	rm -f *~ *core
	rm -f *.xml
	rm -f Makefile.old
	rm -f *.vcproj
	rm -rf Release Debug

#------------------------------------------------------------
# vcproj file build rules
#------------------------------------------------------------
win32_builddir = .

vcproj: [class_name]_vc8.vcproj [class_name]_vc9.vcproj

[class_name]_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \\
		--projectname [class_name]Test \\
		--type EXE \\
		--vcversion "8.00" \\
		--version $(COIL_VERSION) \\
		--out $(win32_builddir)/[class_name]_vc8.vcproj \\
		--yaml ../coil_test.vcproj.yaml \\
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -sm $(win32_builddir)/[class_name]_vc8.vcproj

[class_name]_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \\
		--projectname [class_name]Test \\
		--type EXE \\
		--vcversion "9.00" \\
		--version $(COIL_VERSION) \\
		--out $(win32_builddir)/[class_name]_vc9.vcproj \\
		--yaml ../coil_test.vcproj.yaml \\
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -sm $(win32_builddir)/[class_name]_vc9.vcproj


"""

test_cpp = """// -*- C++ -*-
/*!
 * @file   [class_name]Tests.cpp
 * @brief  [class_name] test class
 * @date   [dollar]Date[dollar]
 
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * [dollar]Id[dollar]
 
 *
 */

/*
 * [dollar]Log[dollar]

 *
 */

#ifndef [class_name]_cpp
#define [class_name]_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class [class_name]Tests class
 * @brief [class_name] test
 */
namespace [class_name]

{
  class [class_name]Tests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE([class_name]Tests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    [class_name]Tests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~[class_name]Tests()
    {
    }
  
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }
  
    /* test case */
    void test_case0()
    {
    }
  };
}; // namespace [class_name]


/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION([class_name]::[class_name]Tests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
    bool retcode = runner.run();
    return !retcode;
}
#endif // MAIN
#endif // [class_name]_cpp
"""


class test_dict:
    def __init__(self, classname):
        self.data = {}
        self.data["dollar"] = "$"
        self.data["class_name"] = classname
        self.data["makefile"]   = classname + "/Makefile.am"
        self.data["testcpp"]    = classname + "/" + classname + "Tests.cpp"
        return
    def get_dict(self):
        return self.data
    

class test_gen:
    def __init__(self, data):
        self.data = data
        return
    
    def gen(self, fname, temp_txt, data):
        f = file(fname, "w")
        t = yat.Template(temp_txt)
        #t.parse(temp_txt)
        text=t.generate(data)
        f.write(text)
        f.close()
        print "\"", fname, "\"" " was generated."
        return

    def gen_all(self):
        self.write_makefile()
        self.write_testcpp()
        return

    def write_makefile(self):
        self.gen(self.data["makefile"], makefile_am, self.data)
        return

    def write_testcpp(self):
        self.gen(self.data["testcpp"], test_cpp, self.data)
        return


if len(sys.argv) < 2:
    sys.exit(1)

class_name = sys.argv[1]
try:
    os.mkdir(class_name, 0755)
except:
    print "Directory \"" + class_name + "\" already exists."
    sys.exit(1)

data = test_dict(class_name)
gen  = test_gen(data.get_dict())
gen.gen_all()
