# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Time unit test
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


noinst_PROGRAMS = TimeTests

TimeTests_COMMON_SOURCES = ../TestRunner.cpp \
                    $(top_srcdir)/common/TimeValue.cpp \
                    TimeTests.cpp
TimeTests_SOURCES = $(TimeTests_COMMON_SOURCES) \
                    $(top_srcdir)/posix/coil/Task.cpp \
                    $(top_srcdir)/posix/coil/Time.cpp 
TimeTests_LDFLAGS = -L$(libdir)
TimeTests_LDADD   = -lcppunit

TEST_SRC = $(TimeTests_COMMON_SOURCES) \
           $(top_srcdir)/win32/coil/Task.cpp \
           $(top_srcdir)/win32/coil/Time.cpp 
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./TimeTests

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

vcproj: Time_vc8.vcproj Time_vc9.vcproj  Time_vc10.vcproj

Time_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname TimeTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Time_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Time_vc8.vcproj

Time_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname TimeTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Time_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Time_vc9.vcproj

Time_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname TimeTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Time_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/Time_vc10.vcproj


