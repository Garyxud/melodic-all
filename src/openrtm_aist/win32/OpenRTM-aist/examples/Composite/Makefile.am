## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist/examples/Composite
##
## $Id: Makefile.am 725 2008-05-13 16:49:04Z n-ando $
##---------------------------------------------------------------------------
clean-local:
	rm -rf Debug Release
	rm -rf *~ *.cpp *.h *.idl *.conf
	rm -rf *.vcproj *.vcxproj *.user *.ncb *.suo
	rm -rf Composite
	rm -rf Controller ControllerComp
	rm -rf Motor MotorComp
	rm -rf Sensor SensorComp

src:
	cp -p $(top_srcdir)/examples/Composite/*.cpp .
	cp -p $(top_srcdir)/examples/Composite/*.h .
	cp -p $(top_srcdir)/examples/Composite/*.conf .
	qkc -O- -ms *.cpp
	qkc -O- -ms *.h
	qkc -O- -ms *.conf

dist-hook: src
