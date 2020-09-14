# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Factory unit test
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

AM_CPPFLAGS= -I../../include      \
	-I$(includedir)           \
	-I$(top_builddir)

AM_LDFLAGS= -L.                   \
	-L$(top_builddir)


noinst_PROGRAMS = FactoryTests

FactoryTests_SOURCES = ../TestRunner.cpp FactoryTests.cpp
FactoryTests_LDFLAGS = -L../../lib/.libs -L$(libdir)
FactoryTests_LDADD   = -lcppunit -lcoil

noinst_LTLIBRARIES = libPluginC.la

libPluginC_la_SOURCES = \
	PluginC.cpp

libPluginC_la_LDFLAGS = \
	-rpath `pwd`

TEST_SRC = $(FactoryTests_SOURCES) \
           $(top_srcdir)/win32/coil/DynamicLib.cpp 
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./FactoryTests

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

vcproj: Factory_vc8.vcproj Factory_vc9.vcproj Factory_vc10.vcproj Factory_libPluginC_vc8.vcproj Factory_libPluginC_vc9.vcproj Factory_libPluginC_vc10.vcproj



Factory_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FactoryTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Factory_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Factory_vc8.vcproj

Factory_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FactoryTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Factory_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Factory_vc9.vcproj

Factory_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FactoryTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Factory_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/Factory_vc10.vcproj

Factory_libPluginC_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FactoryTest_libPluginC \
		--type DLL \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Factory_libPluginC_vc8.vcproj \
		--yaml ./coil_test_dll.vcproj.yaml \
		--source $(libPluginC_la_SOURCES)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Factory_libPluginC_vc8.vcproj


Factory_libPluginC_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FactoryTest_libPluginC  \
		--type DLL \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Factory_libPluginC_vc9.vcproj \
		--yaml ./coil_test_dll.vcproj.yaml \
		--source $(libPluginC_la_SOURCES)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Factory_libPluginC_vc9.vcproj

Factory_libPluginC_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FactoryTest_libPluginC  \
		--type DLL \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Factory_libPluginC_vc10.vcproj \
		--yaml ./coil_test_dll.vcproj.yaml \
		--source $(libPluginC_la_SOURCES)
	qkc -O- -sm $(win32_builddir)/Factory_libPluginC_vc10.vcproj



