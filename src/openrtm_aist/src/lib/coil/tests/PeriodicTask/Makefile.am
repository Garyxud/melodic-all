# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for PeriodicTask unit test
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

AM_CPPFLAGS= -I../../include     \
	-I$(includedir)           \
	-I$(top_builddir)

AM_LDFLAGS= -L.                   \
	-L$(top_builddir)


noinst_PROGRAMS = PeriodicTaskTests

PeriodicTaskTests_SOURCES = ../TestRunner.cpp PeriodicTaskTests.cpp \
           $(top_srcdir)/common/PeriodicTask.cpp
PeriodicTaskTests_LDFLAGS = -L../../lib -L$(libdir)
PeriodicTaskTests_LDADD   = -lcppunit -lcoil

TEST_SRC = $(PeriodicTaskTests_SOURCES) \
           $(top_srcdir)/common/TimeValue.cpp \
           $(top_srcdir)/win32/coil/Task.cpp \
           $(top_srcdir)/common/TimeMeasure.cpp 

TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./PeriodicTaskTests

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

vcproj: PeriodicTask_vc8.vcproj PeriodicTask_vc9.vcproj PeriodicTask_vc10.vcproj

PeriodicTask_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname PeriodicTaskTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/PeriodicTask_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/PeriodicTask_vc8.vcproj

PeriodicTask_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname PeriodicTaskTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/PeriodicTask_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/PeriodicTask_vc9.vcproj

PeriodicTask_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname PeriodicTaskTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/PeriodicTask_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/PeriodicTask_vc10.vcproj


