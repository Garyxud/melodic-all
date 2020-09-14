## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/examples/USBCamera
##
## $Id$
##---------------------------------------------------------------------------
clean-local:
	rm -rf USBCameraAcquire USBCameraAcquireComp
	rm -rf USBCameraMonitor USBCameraMonitorComp
	rm -rf *.vcproj *.user *.ncb *.suo *.sln *.lst *.vcxproj


vcproj: vc8proj vc9proj vc10proj

vc8proj: USBCameraAcquireComp_vc8.vcproj USBCameraAcquire_vc8.vcproj \
	USBCameraMonitorComp_vc8.vcproj USBCameraMonitor_vc8.vcproj

vc9proj: USBCameraAcquireComp_vc9.vcproj USBCameraAcquire_vc9.vcproj\
	USBCameraMonitorComp_vc9.vcproj USBCameraMonitor_vc9.vcproj

vc10proj: USBCameraAcquireComp_vc10.vcxproj USBCameraAcquire_vc10.vcxproj\
	USBCameraMonitorComp_vc10.vcxproj USBCameraMonitor_vc10.vcxproj

USBCameraAcquireComp_vc8.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                        \
		--type RTCEXE                                           \
		--vcversion "8.00"                                      \
		--projectname "USBCameraAcquireComp"                    \
		--version $(RTM_VERSION)                                \
		--out USBCameraAcquireComp_vc8.vcproj                   \
		--source USBCameraAcquire.cpp USBCameraAcquireComp.cpp  \
		--header USBCameraAcquire.h
	qkc -O- -sm USBCameraAcquireComp_vc8.vcproj
USBCameraAcquire_vc8.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                 \
		--type RTCDLL                                    \
		--vcversion "8.00"                               \
		--projectname "USBCameraAcquire"                 \
		--version $(RTM_VERSION)                         \
		--out USBCameraAcquire_vc8.vcproj                \
		--source USBCameraAcquire.cpp                    \
		--header USBCameraAcquire.h
	qkc -O- -sm USBCameraAcquire_vc8.vcproj
USBCameraMonitorComp_vc8.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                        \
		--type RTCEXE                                           \
		--vcversion "8.00"                                      \
		--projectname "USBCameraMonitorComp"                    \
		--version $(RTM_VERSION)                                \
		--out USBCameraMonitorComp_vc8.vcproj                   \
		--source USBCameraMonitor.cpp USBCameraMonitorComp.cpp  \
		--header USBCameraMonitor.h
	qkc -O- -sm USBCameraMonitorComp_vc8.vcproj
USBCameraMonitor_vc8.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                 \
		--type RTCDLL                                    \
		--vcversion "8.00"                               \
		--projectname "USBCameraMonitor"                 \
		--version $(RTM_VERSION)                         \
		--out USBCameraMonitor_vc8.vcproj                \
		--source USBCameraMonitor.cpp                    \
		--header USBCameraMonitor.h
	qkc -O- -sm USBCameraMonitor_vc8.vcproj

USBCameraAcquireComp_vc9.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                        \
		--type RTCEXE                                           \
		--vcversion "9.00"                                      \
		--projectname "USBCameraAcquireComp"                    \
		--version $(RTM_VERSION)                                \
		--out USBCameraAcquireComp_vc9.vcproj                   \
		--source USBCameraAcquire.cpp USBCameraAcquireComp.cpp  \
		--header USBCameraAcquire.h
	qkc -O- -sm USBCameraAcquireComp_vc9.vcproj
USBCameraAcquire_vc9.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                 \
		--type RTCDLL                                    \
		--vcversion "9.00"                               \
		--projectname "USBCameraAcquire"                 \
		--version $(RTM_VERSION)                         \
		--out USBCameraAcquire_vc9.vcproj                \
		--source USBCameraAcquire.cpp                    \
		--header USBCameraAcquire.h
	qkc -O- -sm USBCameraAcquire_vc9.vcproj
USBCameraMonitorComp_vc9.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                       \
		--type RTCEXE                                          \
		--vcversion "9.00"                                     \
		--projectname "USBCameraMonitorComp"                   \
		--version $(RTM_VERSION)                               \
		--out USBCameraMonitorComp_vc9.vcproj                  \
		--source USBCameraMonitor.cpp USBCameraMonitorComp.cpp \
		--header USBCameraMonitor.h
	qkc -O- -sm USBCameraMonitorComp_vc9.vcproj
USBCameraMonitor_vc9.vcproj:
	$(top_srcdir)/build/vcprojtool.py vcproj                 \
		--type RTCDLL                                    \
		--vcversion "9.00"                               \
		--projectname "USBCameraMonitor"                 \
		--version $(RTM_VERSION)                         \
		--out USBCameraMonitor_vc9.vcproj                \
		--source USBCameraMonitor.cpp                    \
		--header USBCameraMonitor.h
	qkc -O- -sm USBCameraMonitor_vc9.vcproj

USBCameraAcquireComp_vc10.vcxproj:
	$(top_srcdir)/build/vcxprojtool.py vcxproj                        \
		--type RTCEXE                                           \
		--vcversion "10.00"                                      \
		--projectname "USBCameraAcquireComp"                    \
		--version $(RTM_VERSION)                                \
		--out USBCameraAcquireComp_vc10.vcxproj                   \
		--source USBCameraAcquire.cpp USBCameraAcquireComp.cpp  \
		--header USBCameraAcquire.h
	qkc -O- -sm USBCameraAcquireComp_vc10.vcxproj
USBCameraAcquire_vc10.vcxproj:
	$(top_srcdir)/build/vcxprojtool.py vcxproj                 \
		--type RTCDLL                                    \
		--vcversion "10.00"                               \
		--projectname "USBCameraAcquire"                 \
		--version $(RTM_VERSION)                         \
		--out USBCameraAcquire_vc10.vcxproj                \
		--source USBCameraAcquire.cpp                    \
		--header USBCameraAcquire.h
	qkc -O- -sm USBCameraAcquire_vc10.vcxproj
USBCameraMonitorComp_vc10.vcxproj:
	$(top_srcdir)/build/vcxprojtool.py vcxproj                       \
		--type RTCEXE                                          \
		--vcversion "10.00"                                     \
		--projectname "USBCameraMonitorComp"                   \
		--version $(RTM_VERSION)                               \
		--out USBCameraMonitorComp_vc10.vcxproj                  \
		--source USBCameraMonitor.cpp USBCameraMonitorComp.cpp \
		--header USBCameraMonitor.h
	qkc -O- -sm USBCameraMonitorComp_vc10.vcxproj
USBCameraMonitor_vc10.vcxproj:
	$(top_srcdir)/build/vcxprojtool.py vcxproj                 \
		--type RTCDLL                                    \
		--vcversion "10.00"                               \
		--projectname "USBCameraMonitor"                 \
		--version $(RTM_VERSION)                         \
		--out USBCameraMonitor_vc10.vcxproj                \
		--source USBCameraMonitor.cpp                    \
		--header USBCameraMonitor.h
	qkc -O- -sm USBCameraMonitor_vc10.vcxproj

sln: USBCamera_vc8.sln USBCamera_vc9.sln USBCamera_vc10.sln

USBCamera_vc8.sln: vc8proj
	$(top_srcdir)/build/slntool.py  \
		--vcversion VC8         \
		--out USBCamera_vc8.sln \
		`find ./ -name '*_vc8.vcproj'`
	qkc -O- -ms USBCamera_vc8.sln

USBCamera_vc9.sln: vc9proj
	$(top_srcdir)/build/slntool.py  \
		--vcversion VC9         \
		--out USBCamera_vc9.sln \
		`find ./ -name '*_vc9.vcproj'`
	qkc -O- -ms USBCamera_vc9.sln

USBCamera_vc10.sln: vc10proj
	$(top_srcdir)/build/slntool.py  \
		--vcversion VC10         \
		--out USBCamera_vc10.sln \
		`find ./ -name '*_vc10.vcxproj'`
	qkc -O- -ms USBCamera_vc10.sln

dist-hook: vcproj sln lst

EXTRA_DIST =                             \
	user_config.vsprops              \
	USBCameraAcquire.cpp             \
	USBCameraAcquire.h               \
	USBCameraAcquireComp.cpp         \
	USBCameraMonitor.cpp             \
	USBCameraMonitor.h               \
	USBCameraMonitorComp.cpp         \
	USBCameraAcquire_vc8.vcproj      \
	USBCameraAcquireComp_vc8.vcproj  \
	USBCameraAcquire_vc9.vcproj      \
	USBCameraAcquireComp_vc9.vcproj  \
	USBCameraAcquire_vc10.vcxproj      \
	USBCameraAcquireComp_vc10.vcxproj  \
	USBCameraMonitor_vc8.vcproj      \
	USBCameraMonitorComp_vc8.vcproj  \
	USBCameraMonitor_vc9.vcproj      \
	USBCameraMonitorComp_vc9.vcproj  \
	USBCameraMonitor_vc10.vcxproj      \
	USBCameraMonitorComp_vc10.vcxproj  \
	USBCamera_vc8.sln                \
	USBCamera_vc9.sln                \
	USBCamera_vc10.sln                \
	rtc.conf

lst:
	echo $(EXTRA_DIST) > src.lst

