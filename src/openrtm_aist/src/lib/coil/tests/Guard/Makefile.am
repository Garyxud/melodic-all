# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Guard unit test
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


noinst_PROGRAMS = GuardTests

GuardTests_COMMON_SOURCES = ../TestRunner.cpp \
                             GuardTests.cpp
GuardTests_SOURCES = $(GuardTests_COMMON_SOURCES) \
                     $(top_srcdir)/posix/coil/Task.cpp 
GuardTests_LDFLAGS = -L$(libdir)
GuardTests_LDADD   = -lcppunit

TEST_SRC = $(GuardTests_COMMON_SOURCES) \
           $(top_srcdir)/win32/coil/Task.cpp
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./GuardTests

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

vcproj: Guard_vc8.vcproj Guard_vc9.vcproj Guard_vc10.vcproj

Guard_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname GuardTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Guard_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Guard_vc8.vcproj

Guard_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname GuardTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Guard_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Guard_vc9.vcproj

Guard_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname GuardTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Guard_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/Guard_vc10.vcproj


