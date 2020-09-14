## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for libRTC
##
## $Id: Makefile.am 1087 2008-12-19 06:20:06Z n-ando $
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

bin_SCRIPTS = rtm-manager

IDLC = @IDLC@
IDLFLAGS = -bpython
IDLS =          \
	SDOPackage.idl \
	RTC.idl        \
	OpenRTM.idl    \
	Manager.idl

SUFFIXES = .idl _idl.py

managerhelperdir = $(rtm_libdir)/manager_helper

managerhelper_SCRIPTS = \
	NSHelper.py        \
	$(IDLS:.idl=_idl.py)

$(IDLS):
	cp -p $(top_srcdir)/src/lib/rtm/idl/SDOPackage.idl .
	cp -p $(top_srcdir)/src/lib/rtm/idl/RTC.idl .
	cp -p $(top_srcdir)/src/lib/rtm/idl/OpenRTM.idl .
	cp -p $(top_srcdir)/src/lib/rtm/idl/Manager.idl .

.idl_idl.py:
	$(IDLC) $(IDLFLAGS) $<

EXTRA_DIST = $(IDLS:.idl=_idl.py)
EXTRA_DIST += rtm-manager NSHelper.py

clean-local:
	rm -rf OpenRTM RTC RTM SDOPackage
	rm -rf OpenRTM__POA RTC__POA RTM__POA SDOPackage__POA
	rm -f *.pyc
	rm -f *_idl.py
	rm -f *.idl
	rm -f *~


