# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for stringutil unit test
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


noinst_PROGRAMS = stringutilTests

stringutilTests_SOURCES = ../TestRunner.cpp stringutilTests.cpp
stringutilTests_LDFLAGS = -L../../lib -L$(libdir)
stringutilTests_LDADD   = -lcppunit -lcoil

TEST_SRC = $(stringutilTests_SOURCES)
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./stringutilTests

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

vcproj: stringutil_vc8.vcproj stringutil_vc9.vcproj stringutil_vc10.vcproj

stringutil_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname stringutilTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/stringutil_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/stringutil_vc8.vcproj

stringutil_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname stringutilTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/stringutil_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/stringutil_vc9.vcproj

stringutil_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname stringutilTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/stringutil_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/stringutil_vc10.vcproj


