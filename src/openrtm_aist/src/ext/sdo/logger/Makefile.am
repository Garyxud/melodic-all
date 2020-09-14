## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for Component logger SDO service
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

IDLC = @IDLC@
IDLFLAGS = @IDL_FLAGS@ -I$(top_srcdir)/src/lib/rtm/idl
LIBS = @LIBS@

AM_CPPFLAGS=-I$(top_srcdir)/src/lib -I$(top_srcdir)/src/lib/coil/include -I$(top_srcdir)/src/lib/rtm/idl
AM_LDFLAGS=-L$(top_builddir) -L$(top_builddir)/src/lib/rtm -L$(top_builddir)/src/lib/rtm/idl

#------------------------------------------------------------
# Some rules for IDL compile
#------------------------------------------------------------
SUFFIXES = .o .so .idl Skel.cpp Stub.cpp Stub.o Skel.o

WRAPPER  = $(top_builddir)/utils/rtm-skelwrapper/rtm-skelwrapper
WRAPPER_FLAGS = --include-dir="" --skel-suffix=Skel --stub-suffix=Stub

.idlSkel.cpp:
	echo $(PROVIDER_OBJ)
	$(IDLC) $(IDLFLAGS) $<
	PYTHONPATH="$(top_srcdir)/build:$(PYTHONPATH)" \
	PATH="$(top_srcdir)/utils/rtm-config:$(PATH)"  \
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=$<

.idlStub.cpp:
	$(IDLC) $(IDLFLAGS) $<
	PYTHONPATH="$(PYTHONPATH):$(top_srcdir)/build" \
	PATH="$(top_srcdir)/utils/rtm-config:$(PATH)"  \
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=$<

#------------------------------------------------------------
# Source list
#------------------------------------------------------------
IDL_SOURCES = Logger.idl

LOGGER_DIST_SRC   = LoggerConsumer.cpp
LOGGER_NODIST_SRC = $(IDL_SOURCES:.idl=Stub.cpp)
LOGGER_SRC        = $(LOGGER_DIST_SRC) $(LOGGER_NODIST_SRC)

LOGGER_DIST_H     = $(LOGGER_DIST_SRC:.cpp=.h)
LOGGER_NODIST_H   = $(LOGGER_NODIST_SRC:.cpp=.h)
LOGGER_H          = $(LOGGER_DIST_H) $(LOGGER_NODIST_H)

#------------------------------------------------------------
# Object list
#------------------------------------------------------------
LOGGER_OBJ=$(LOGGER_DIST_SRC:.cpp=.o)

#------------------------------------------------------------
# Targets
#------------------------------------------------------------
# Targets directories
socompdir = $(rtm_sdodir)

# Loadable Module
noinst_LTLIBRARIES = libLoggerSkel.la libLoggerStub.la 
nodist_libLoggerSkel_la_SOURCES = $(IDL_SOURCES:.idl=Skel.cpp)
nodist_libLoggerStub_la_SOURCES = $(IDL_SOURCES:.idl=Stub.cpp)

socomp_LTLIBRARIES = LoggerConsumer.la

dist_LoggerConsumer_la_SOURCES = \
	$(LOGGER_DIST_SRC) $(LOGGER_DIST_H)
LoggerConsumer_la_LDFLAGS = -module
LoggerConsumer_la_DEPENDENCIES = libLoggerStub.la
LoggerConsumer_la_LIBADD  =                \
	libLoggerStub.la                   \
	$(top_builddir)/src/lib/rtm/libRTC.la         \
	$(top_builddir)/src/lib/coil/lib/libcoil.la

#------------------------------------------------------------
# Extra distributed files
#------------------------------------------------------------
EXTRA_DIST = $(IDL_SOURCES)

ALL_SRC = $(LOGGER_SRC)

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(ALL_SRC) > src.lst
	echo $(EXTRA_DIST) > other.lst

dist-hook: lst


clean_objs:
	rm -f $(OBJS)

clean_skelstub:
	rm -f *Skel.h *Skel.cpp
	rm -f *Stub.h *Stub.cpp
	rm -f *.hh *SK.cc
	rm -f *C.cpp *C.h *S.cpp *S.h *_T.cpp *_T.h *.inl

clean-local: clean_objs clean_skelstub
	rm -f *.bak *.rpo *.sym lib*.*_pure_*
	rm -f *.lst *core *~ *loT *.log
	rm -f Makefile.old 
