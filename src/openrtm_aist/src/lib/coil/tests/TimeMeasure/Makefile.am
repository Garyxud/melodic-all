# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for TimeMeasure unit test
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

AM_CPPFLAGS= -I$(top_builddir)/include \
	-I$(includedir)                \
	-I$(top_builddir)

AM_LDFLAGS= -L.                   \
	-L$(top_builddir)


noinst_PROGRAMS = TimeMeasureTests

TimeMeasureTests_SOURCES = ../TestRunner.cpp TimeMeasureTests.cpp
TimeMeasureTests_LDFLAGS = -L$(libdir) -L$(top_srcdir)/lib
TimeMeasureTests_LDADD   = -lcppunit -lcoil

TEST_SRC = $(TimeMeasureTests_SOURCES) \
	$(top_srcdir)/common/TimeMeasure.cpp \
	$(top_srcdir)/win32/coil/Time.cpp \
	$(top_srcdir)/common/TimeValue.cpp

TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./TimeMeasureTests

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

vcproj: TimeMeasure_vc8.vcproj TimeMeasure_vc9.vcproj

TimeMeasure_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname TimeMeasureTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/TimeMeasure_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/TimeMeasure_vc8.vcproj

TimeMeasure_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname TimeMeasureTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/TimeMeasure_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/TimeMeasure_vc9.vcproj


