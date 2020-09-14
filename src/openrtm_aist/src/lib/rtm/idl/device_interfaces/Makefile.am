## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for device interface files
##
## $Id: Makefile.am 1223 2009-02-27 07:56:06Z n-ando $
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

IDL_SOURCES = ActArray.idl \
	Bumper.idl             \
	DIO.idl                \
	Fiducial.idl           \
	Gripper.idl            \
	INS.idl                \
	Limb.idl               \
	Multicamera.idl        \
	Paths.idl              \
	Position.idl           \
	RFID.idl               \
	AIO.idl                \
	Camera.idl             \
	GPS.idl                \
	IMU.idl                \
	Map.idl                \
	PanTilt.idl            \
	PointCloud.idl         \
	Ranger.idl

# Install directory of device idl files
idldir       = $(rtm_includedir)/rtm/idl/device_interfaces

idl_DATA = $(IDL_SOURCES)


#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(IDL_SOURCES) > idls.lst

# Extra sources included distribution
EXTRA_DIST = \
	$(IDL_SOURCES)

