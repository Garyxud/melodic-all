# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Condition unit test
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
	-I$(top_builddir)         \
	-I$(top_srcdir)/include   \
	-I$(includedir)

AM_LDFLAGS= -L../../lib/.libs     \
	-L$(top_builddir)


noinst_PROGRAMS = ConditionTests

ConditionTests_COMMON_SOURCES = ../TestRunner.cpp \
                                ConditionTests.cpp
ConditionTests_SOURCES = $(ConditionTests_COMMON_SOURCES)
ConditionTests_LDFLAGS = -L../../lib/.libs -L$(libdir)
ConditionTests_LDADD   = -lcppunit -lcoil

TEST_SRC = $(ConditionTests_COMMON_SOURCES) \
	$(top_srcdir)/win32/coil/Task.cpp 

TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./ConditionTests

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

vcproj: Condition_vc8.vcproj Condition_vc9.vcproj Condition_vc10.vcproj 

Condition_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname ConditionTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Condition_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Condition_vc8.vcproj

Condition_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname ConditionTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Condition_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Condition_vc9.vcproj

Condition_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname ConditionTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Condition_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/Condition_vc10.vcproj


