# -*- Makefile -*-
#------------------------------------------------------------
# @file   Makefile.am
# @brief  Makefile.am for Properties unit test
# @date   $Date: 2007/01/21 06:52:57 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# $Id$
#
#------------------------------------------------------------

#
# $Log: Makefile.am,v $
# Revision 1.3  2007/01/21 06:52:57  n-ando
# The include search path option order in AM_CPPFLAGS was changed.
#
# Revision 1.2  2007/01/12 14:44:49  n-ando
# Some fixes for distribution control.
#
# Revision 1.1  2006/11/27 08:29:54  n-ando
# TestSuites are devided into each directory.
#
#


AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@
LIBS = @LIBS@

AM_CPPFLAGS =             \
	 -I.              \
	-I$(top_builddir) \
	-I$(includedir)   \
	-I$(top_srcdir)/include


AM_LDFLAGS =              \
	-L.               \
	-L$(top_builddir)


noinst_PROGRAMS = PropertiesTests

nodist_PropertiesTests_SOURCES =           \
	../TestRunner.cpp                  \
	$(top_builddir)/common/Properties.cpp \
	$(top_builddir)/common/stringutil.cpp \
	PropertiesTests.cpp

PropertiesTests_LDFLAGS = -L$(libdir)
PropertiesTests_LDADD   = -lcppunit

EXTRA_DIST =                   \
	PropertiesTests.cpp    \
	defaults.conf          \
	storedRead.conf        \
	splitKeyValueTest.data \
	store-test.data

TEST_SRC = $(nodist_PropertiesTests_SOURCES)
TEST_H   = 

# all
all: do-test

# do tests
do-test:
	./PropertiesTests

# --xml propertiestests.xml \
#	--xsl default.xml

# clean-up
clean-local:
	rm -f                               \
	new.conf stored.conf                \
	*.o *.Po *.gch *.la                 \
	*Skel.cpp *Skel.h *Stub.cpp *Stub.h \
	*~ *core                            \
	Makefile.old                        \
	rm -f *.vcproj                      \
	*.cc *.hh *.i *.a *.c *.inl

#------------------------------------------------------------
# vcproj file build rules
#------------------------------------------------------------
win32_builddir = .

vcproj: Properties_vc8.vcproj Properties_vc9.vcproj Properties_vc10.vcproj

Properties_vc8.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname PropertiesTaskTest \
		--type EXE \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Properties_vc8.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Properties_vc8.vcproj

Properties_vc9.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname PropertiesTest \
		--type EXE \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Properties_vc9.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
#		--header $(TEST_H)
	qkc -O- -sm $(win32_builddir)/Properties_vc9.vcproj

Properties_vc10.vcproj:
	$(top_builddir)/build/vcprojtool.py vcproj \
		--projectname PropertiesTest \
		--type EXE \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/Properties_vc10.vcproj \
		--yaml ../coil_test.vcproj.yaml \
		--source $(TEST_SRC)
	qkc -O- -sm $(win32_builddir)/Properties_vc10.vcproj





