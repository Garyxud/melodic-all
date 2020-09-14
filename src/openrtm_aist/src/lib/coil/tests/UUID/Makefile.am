# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for UUID unit test
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


noinst_PROGRAMS = UUIDTests

UUIDTests_COMMON_SOURCES = ../TestRunner.cpp \
                           UUIDTests.cpp
UUIDTests_SOURCES = $(UUIDTests_COMMON_SOURCES) \
                    $(top_srcdir)/posix/coil/UUID.cpp
UUIDTests_LDFLAGS = -L$(libdir)
UUIDTests_LDADD   = -lcppunit -luuid

TEST_SRC = $(UUIDTests_COMMON_SOURCES) \
           $(top_srcdir)/win32/coil/UUID.cpp
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./UUIDTests

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

vcproj: UUID_vc8.vcproj UUID_vc9.vcproj UUID_vc10.vcproj

UUID_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname UUIDTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/UUID_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/UUID_vc8.vcproj

UUID_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname UUIDTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/UUID_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/UUID_vc9.vcproj

UUID_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname UUIDTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/UUID_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/UUID_vc10.vcproj

