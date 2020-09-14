# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for OS unit test
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


noinst_PROGRAMS = OSTests

OSTests_SOURCES = ../TestRunner.cpp \
                  OSTests.cpp
OSTests_LDFLAGS = -L$(libdir)
OSTests_LDADD   = -lcppunit 

TEST_SRC = $(OSTests_SOURCES)
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./OSTests

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

vcproj: OS_vc8.vcproj OS_vc9.vcproj OS_vc10.vcproj

OS_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname OSTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/OS_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/OS_vc8.vcproj

OS_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname OSTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/OS_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/OS_vc9.vcproj

OS_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname OSTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/OS_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/OS_vc10.vcproj


