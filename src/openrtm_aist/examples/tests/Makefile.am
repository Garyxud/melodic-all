## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for tests
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

# IDL and omniidl path and options
IDLDIR = $(top_srcdir)/rtm/idl/
IDLC = omniidl
IDLPYFLAGS = -bpython -nf

VPATH = $(IDLDIR)

# Suffix and suffix rule
SUFFIXES = .idl _idl.py 
.idl_idl.py:
	$(IDLC) $(IDLPYFLAGS) $(IDLDIR)$<


# tests script to be installed bin directory
bin_SCRIPTS = tests

tests:

# Something to install to libdir
pymoduledir = $(rtm_libdir)/tests
pymodule_SCRIPTS = \
	$(PY_STUBS) \
	$(TESTS_SOURCES)

# tests python sources
TESTS_SOURCES = \
	AddRemoveMemberSDOPackageTest.py \
	AddRemoveOrganizationSDOPackageTest.py \
	AddRemoveRTCTest.py \
	AttachDetachRTCTest.py \
	ConfigurationSDOPackageTest.py \
	ConnectRTCTest.py \
	CreateDeleteRTCTest.py \
	DataPortTest.py \
	ManagerTest.py \
	OrganizationSDOPackageTest.py \
	RTCTest.py \
	SDOPackageTest.py \
	ServiceProfileSDOPackageTest.py \
	SetMemberSDOPackageTest.py \
	rtc_handle.py \
	CorbaNaming.py


py_stub: $(PY_STUBS)

dist-hook: py_stub

EXTRA_DIST =           \
	$(TESTS_SOURCES) \
	AddRemoveMemberSDOPackageTest.sh \
	AddRemoveOrganizationSDOPackageTest.sh \
	AddRemoveRTCTest.sh \
	AttachDetachRTCTest.sh \
	ConfigurationSDOPackageTest.sh \
	ConnectRTCTest.sh \
	CreateDeleteRTCTest.sh \
	DataPortTest.sh \
	ManagerTest.sh \
	OrganizationSDOPackageTest.sh \
	RTCTest.sh \
	SDOPackageTest.sh \
	ServiceProfileSDOPackageTest.sh \
	SetMemberSDOPackageTest.sh \
	run.sh \
	setup.sh \
	rtc.conf



# all
all: do-test

# do tests
do-test:
	./setup.sh


clean-local:
	rm -rf \
	*~ *core *.pyc *.idl *_idl.py *.bak *.log *.so .deps .libs rtcd rtm-naming \
	OpenRTM OpenRTM__POA RTC RTC__POA RTM RTM__POA SDOPackage SDOPackage__POA

