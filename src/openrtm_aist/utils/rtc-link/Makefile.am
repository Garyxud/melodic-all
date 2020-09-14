## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for rtc-link
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


# rtc-link script to be installed bin directory
bin_SCRIPTS = rtc-link

rtc-link:

# Something to install to libdir
pymoduledir = $(rtm_libdir)/rtc-link
pymodule_SCRIPTS = \
	$(PY_STUBS) \
	$(RTCLINK_SOURCES)

# RTCLink python sources
RTCLINK_SOURCES = \
	RtcLink.py \
	RtcLink.pyw \
	RtmAbout.py \
	RtmNSHelper.py \
	RtmProfileList.py \
	RtmCompData.py \
	RtmLineUtil.py \
	RtmSystemDraw.py \
	RtmFrame.py \
	RtmParser.py \
	RtmTreeCtrl.py \
	rtimages.py \
	RtmDialog.py \
	RtmDtdValidator.py \
	rtc-link

# rule for installing RTC RTC__POA SDOPackage SDOPackage__POA directories
install-stubs:
	if test -d RTC; then \
	  $(mkinstalldirs) $(DESTDIR)$(pymoduledir)/RTC; \
	  for x in RTC/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(pymoduledir)/RTC/ ;\
	    fi \
	  done \
	fi
	if test -d RTC__POA; then \
	  $(mkinstalldirs) $(DESTDIR)$(pymoduledir)/RTC__POA; \
	  for x in RTC__POA/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(pymoduledir)/RTC__POA/ ;\
	    fi \
	  done \
	fi
	if test -d SDOPackage; then \
	  $(mkinstalldirs) $(DESTDIR)$(pymoduledir)/SDOPackage; \
	  for x in SDOPackage/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(pymoduledir)/SDOPackage/ ;\
	    fi \
	  done \
	fi
	if test -d SDOPackage__POA; then \
	  $(mkinstalldirs) $(DESTDIR)$(pymoduledir)/SDOPackage__POA; \
	  for x in SDOPackage__POA/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(pymoduledir)/SDOPackage__POA/ ;\
	    fi \
	  done \
	fi


install-data-local: install-stubs


IDL_SOURCES =       \
	SDOPackage.idl	\
	RTC.idl 	    


#
# RTM idl stubs for python
PY_STUBS = $(IDL_SOURCES:.idl=_idl.py)

#RTMBase_idl.py: RTMBase.idl

py_stub: $(PY_STUBS)

dist-hook: py_stub

EXTRA_DIST =           \
	$(RTCLINK_SOURCES) \
	$(PY_STUBS)        \
	RTC                \
	RTC__POA           \
	SDOPackage         \
	SDOPackage__POA


clean-local:
	rm -rf *~ *core *.pyc *_idl.py RTC RTC__POA SDOPackage SDOPackage__POA
