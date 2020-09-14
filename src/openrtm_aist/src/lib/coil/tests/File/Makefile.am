# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for File unit test
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
	-I$(top_builddir)/include \
	-I$(includedir)


AM_LDFLAGS= -L.                   \
	-L$(top_builddir)


noinst_PROGRAMS = FileTests

FileTests_SOURCES = ../TestRunner.cpp FileTests.cpp
FileTests_LDFLAGS = -L$(libdir)
FileTests_LDADD   = -lcppunit

TEST_SRC = $(FileTests_SOURCES)
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./FileTests

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

vcproj: File_vc8.vcproj File_vc9.vcproj File_vc10.vcproj

File_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FileTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/File_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/File_vc8.vcproj

File_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FileTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/File_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/File_vc9.vcproj

File_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname FileTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/File_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/File_vc10.vcproj



