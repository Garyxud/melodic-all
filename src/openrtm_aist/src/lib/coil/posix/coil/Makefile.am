## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for coil (Component Object Infrastructure Layer)
##
## $Id$
##---------------------------------------------------------------------------

AM_CPPFLAGS=-I.. -I$(top_srcdir)/include
DEFAULT_INCLUDES=

# posix API dependent sources
COIL_PLATFORM_SRC =    \
	Condition.cpp  \
	DynamicLib.cpp \
	Mutex.cpp      \
	Process.cpp    \
	Routing.cpp    \
	Signal.cpp     \
	Task.cpp       \
	Time.cpp       \
	UUID.cpp

# posix API dependent headers
COIL_PLATFORM_H   =    \
	atomic.h       \
	File.h         \
	OS.h           \
	$(COIL_PLATFORM_SRC:.cpp=.h)

# coil common source
COIL_COMMON_SRC =        \
	Allocator.cpp    \
	PeriodicTask.cpp \
	Properties.cpp   \
	TimeMeasure.cpp  \
	TimeValue.cpp    \
	Timer.cpp        \
	crc.cpp          \
	stringutil.cpp

# coil common headers
COIL_COMMON_H   =           \
	Allocator.h         \
	Async.h             \
	Factory.h           \
	Guard.h             \
	Listener.h          \
	Logger.h            \
	NonCopyable.h       \
	PeriodicTask.h      \
	PeriodicTaskBase.h  \
	Properties.h        \
	Singleton.h         \
	TimeMeasure.h       \
	TimeValue.h         \
	Timer.h             \
	crc.h               \
	stringutil.h


# coil all sources and headers
COIL_SRC = $(COIL_COMMON_SRC) $(COIL_PLATFORM_SRC)
COIL_H   = $(COIL_COMMON_H) $(COIL_PLATFORM_H)
COIL_ALL = $(COIL_SRC) $(COIL_H)

$(COIL_COMMON_SRC): $(COIL_COMMON_H)
	cp -p $(top_srcdir)/common/$@ .

$(COIL_COMMON_H):
	cp -p $(top_srcdir)/common/$@ .


coilheaderdir = $(coil_includedir)/coil

coilheader_HEADERS = $(COIL_H) $(top_srcdir)/include/coil/config_coil.h

lib_LTLIBRARIES = libcoil.la
#noinst_LTLIBRARIES = libcoil.la

libcoil_la_SOURCES =          \
	$(COIL_ALL)

libcoil_la_LDFLAGS =                \
	-no-undefined               \
	-release $(PACKAGE_VERSION)

clean-local:
	rm -f *~
	rm -f *.o *.lo *.loT
	rm -f $(COIL_COMMON_SRC) $(COIL_COMMON_H)
	chmod 644 *.cpp *.h
