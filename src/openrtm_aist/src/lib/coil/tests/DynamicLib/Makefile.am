# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for DynamicLib unit test
# @date   $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

#
# $Log$
#


AUTOMAKE_OPTIONS = 1.9

AM_CPPFLAGS= -I.                  \
	-I$(includedir)           \
	-I$(top_builddir)         \
	-I$(top_srcdir)/include

AM_LDFLAGS= -L.                   \
	-L$(top_builddir)


noinst_PROGRAMS = DynamicLibTests

DynamicLibTests_COMMON_SOURCES = ../TestRunner.cpp \
                                 DynamicLibTests.cpp
DynamicLibTests_SOURCES = $(DynamicLibTests_COMMON_SOURCES) \
                          $(top_srcdir)/posix/coil/DynamicLib.cpp
DynamicLibTests_LDFLAGS = -L$(libdir) -L../../lib/.libs -L$(libdir)
DynamicLibTests_LDADD   = -lcppunit

noinst_LTLIBRARIES = libDynamicLib.la

libDynamicLib_la_SOURCES = \
	libDll.h \
	libDll.cpp

libDynamicLib_la_LDFLAGS = \
	-rpath `pwd`
#	-module

TEST_SRC = $(DynamicLibTests_COMMON_SOURCES) \
           $(top_srcdir)/win32/coil/DynamicLib.cpp
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./DynamicLibTests

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

vcproj: DynamicLib_vc8.vcproj DynamicLib_vc9.vcproj DynamicLib_vc10.vcproj DynamicLib_libDynamicLib_vc8.vcproj DynamicLib_libDynamicLib_vc9.vcproj DynamicLib_libDynamicLib_vc10.vcproj

DynamicLib_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname DynamicLibTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/DynamicLib_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/DynamicLib_vc8.vcproj

DynamicLib_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname DynamicLibTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/DynamicLib_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/DynamicLib_vc9.vcproj

DynamicLib_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname DynamicLibTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/DynamicLib_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/DynamicLib_vc10.vcproj

DynamicLib_libDynamicLib_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname DynamicLibTestDll \
		--type DLL \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/DynamicLib_libDynamicLib_vc8.vcproj \
		--yaml ../coil_test_dll.vcproj.yaml \
		--source $(libDynamicLib_la_SOURCES)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/DynamicLib_libDynamicLib_vc8.vcproj


DynamicLib_libDynamicLib_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname DynamicLibTestDll  \
		--type DLL \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/DynamicLib_libDynamicLib_vc9.vcproj \
		--yaml ../coil_test_dll.vcproj.yaml \
		--source $(libDynamicLib_la_SOURCES)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/DynamicLib_libDynamicLib_vc9.vcproj


DynamicLib_libDynamicLib_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname DynamicLibTestDll  \
		--type DLL \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/DynamicLib_libDynamicLib_vc10.vcproj \
		--yaml ../coil_test_dll.vcproj.yaml \
		--source $(libDynamicLib_la_SOURCES)
	qkc -O- -sm $(win32_builddir)/DynamicLib_libDynamicLib_vc10.vcproj

