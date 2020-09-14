## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for coil (Component Object Infrastructure Layer)
##
## $Id$
##---------------------------------------------------------------------------

# Win32 API dependent sources
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

# Win32 API dependent headers
COIL_PLATFORM_H =      \
	config_coil.h  \
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


$(COIL_COMMON_SRC):
	cp -p $(top_srcdir)/common/*.cpp .
	qkc -O- -ms *.cpp

$(COIL_COMMON_H):
	cp -p $(top_srcdir)/common/*.h .
	qkc -O- -ms *.h


clean-local:
	rm -f *~
	rm -f $(COIL_COMMON_SRC) $(COIL_COMMON_H)
	rm -f *.vcproj
	rm -f *.vcxproj
	rm -f *.user
	rm -f *.def
	rm -f *.lst
	rm -rf Release Debug

EXTRA_DIST =                \
	$(COIL_ALL)         \
	libcoil_vc8.vcproj  \
	libcoil_vc9.vcproj  \
	libcoil_vc10.vcxproj  \
	libcoil.vcproj.yaml

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(COIL_SRC) > src.lst
	echo $(COIL_H) > header.lst

#------------------------------------------------------------
# vcproj file build rules
#------------------------------------------------------------
win32_builddir = .

vcproj: libcoil_vc8.vcproj libcoil_vc9.vcproj libcoil_vc10.vcxproj libcoil_vc11.vcxproj libcoil_vc12.vcxproj libcoil_vc14.vcxproj

libcoil_vc8.vcproj: libcoil.vcproj.yaml $(COIL_ALL)
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type DLL \
		--vcversion "8.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/libcoil_vc8.vcproj \
		--yaml libcoil.vcproj.yaml \
		--source $(COIL_SRC) \
		--header $(COIL_H)
	qkc -O- -sm $(win32_builddir)/libcoil_vc8.vcproj

libcoil_vc9.vcproj: libcoil.vcproj.yaml $(COIL_ALL)
	$(top_builddir)/build/vcprojtool.py vcproj \
		--type DLL \
		--vcversion "9.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/libcoil_vc9.vcproj \
		--yaml libcoil.vcproj.yaml \
		--source $(COIL_SRC) \
		--header $(COIL_H)
	qkc -O- -sm $(win32_builddir)/libcoil_vc9.vcproj

libcoil_vc10.vcxproj: libcoil.vcproj.yaml $(COIL_ALL)
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "10.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/libcoil_vc10.vcxproj \
		--yaml libcoil.vcproj.yaml \
		--source $(COIL_SRC) \
		--header $(COIL_H)
	qkc -O- -sm $(win32_builddir)/libcoil_vc10.vcxproj

libcoil_vc11.vcxproj: libcoil.vcproj.yaml $(COIL_ALL)
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "11.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/libcoil_vc11.vcxproj \
		--yaml libcoil.vcproj.yaml \
		--source $(COIL_SRC) \
		--header $(COIL_H)
	qkc -O- -sm $(win32_builddir)/libcoil_vc11.vcxproj

libcoil_vc12.vcxproj: libcoil.vcproj.yaml $(COIL_ALL)
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "12.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/libcoil_vc12.vcxproj \
		--yaml libcoil.vcproj.yaml \
		--source $(COIL_SRC) \
		--header $(COIL_H)
	qkc -O- -sm $(win32_builddir)/libcoil_vc12.vcxproj
	
libcoil_vc14.vcxproj: libcoil.vcproj.yaml $(COIL_ALL)
	$(top_builddir)/build/vcxprojtool.py vcxproj \
		--type DLL \
		--vcversion "14.00" \
		--version $(COIL_VERSION) \
		--out $(win32_builddir)/libcoil_vc14.vcxproj \
		--yaml libcoil.vcproj.yaml \
		--source $(COIL_SRC) \
		--header $(COIL_H)
	qkc -O- -sm $(win32_builddir)/libcoil_vc14.vcxproj

#------------------------------------------------------------
# for wxs file
#------------------------------------------------------------
wxs:
	$(top_srcdir)/../../../build/makewxs.py flist \
		-c COILs \
		-o $(top_srcdir)/../../../win32/OpenRTM-aist/installer/coils.yaml \
		-p ..\\coil \
		$(COIL_H) $(COIL_SRC)

dist-hook: lst vcproj wxs

