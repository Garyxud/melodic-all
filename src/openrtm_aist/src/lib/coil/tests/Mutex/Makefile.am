# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Mutex unit test
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


noinst_PROGRAMS = MutexTests

MutexTests_COMMON_SOURCES = ../TestRunner.cpp \
                            MutexTests.cpp

MutexTests_SOURCES = $(MutexTests_COMMON_SOURCES) \
                     $(top_srcdir)/posix/coil/Task.cpp \
                     $(top_srcdir)/posix/coil/Mutex.cpp
MutexTests_LDFLAGS = -L$(libdir)
MutexTests_LDADD   = -lcppunit

TEST_SRC = $(MutexTests_COMMON_SOURCES) \
           $(top_srcdir)/win32/coil/Task.cpp \
           $(top_srcdir)/win32/coil/Mutex.cpp
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./MutexTests

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

vcproj: Mutex_vc8.vcproj Mutex_vc9.vcproj Mutex_vc10.vcproj

Mutex_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname MutexTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Mutex_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Mutex_vc8.vcproj

Mutex_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname MutexTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Mutex_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Mutex_vc9.vcproj

Mutex_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname MutexTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Mutex_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Mutex_vc10.vcproj


