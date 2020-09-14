# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for AsyncInvoker unit test
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

AM_LDFLAGS= -L../../lib           \
	-L$(top_builddir)


noinst_PROGRAMS = AsyncInvokerTests

AsyncInvokerTests_SOURCES = ../TestRunner.cpp AsyncInvokerTests.cpp
AsyncInvokerTests_LDFLAGS = -L../../lib -L$(libdir) 
AsyncInvokerTests_LDADD   = -lcppunit -lcoil

TEST_SRC = $(AsyncInvokerTests_SOURCES) \
           $(top_srcdir)/win32/coil/Task.cpp

TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./AsyncInvokerTests

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

vcproj: AsyncInvoker_vc8.vcproj AsyncInvoker_vc9.vcproj

AsyncInvoker_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname AsyncInvokerTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/AsyncInvoker_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/AsyncInvoker_vc8.vcproj

AsyncInvoker_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname AsyncInvokerTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/AsyncInvoker_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/AsyncInvoker_vc9.vcproj


