## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for template components
##
## $Id:
##------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

TEMPLATES = ActArray.xml \
	AIO.xml \
	Bumper.xml \
	Camera.xml \
	CarlikeControl.xml \
	DIO.xml \
	FeatureMap.xml \
	Fiducial.xml \
	GPS.xml \
	Gripper.xml \
	HeadingControl2D.xml \
	HeadingControl3D.xml \
	IMU.xml \
	INS.xml \
	Joystick.xml \
	Limb.xml \
	Localise2D.xml \
	Localise3D.xml \
	Makefile.am \
	Multicamera.xml \
	OccupancyGridMap.xml \
	Odometry2D.xml \
	Odometry3D.xml \
	PanTilt.xml \
	PathFollower2D.xml \
	PathFollower3D.xml \
	PathPlanner2D.xml \
	PathPlanner3D.xml \
	PointCloud.xml \
	PoseControl2D.xml \
	PoseControl3D.xml \
	Power.xml \
	Ranger.xml \
	RFID.xml \
	VelocityControl2D.xml \
	VelocityControl3D.xml

# Install directory of headers and idl files
exampledir = $(rtm_exampledir)
templatedir = $(exampledir)/templates/

template_DATA = $(TEMPLATES)

#------------------------------------------------------------
# File list for deb/ports packages
#------------------------------------------------------------
lst:
	echo $(TEMPLATES) > templates.lst

EXTRA_DIST = $(TEMPLATES)

