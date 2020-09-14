# Strings used in the rocon framework.

################################ Versions ####################################

# Rocon versions come from the names of classical composers.
#
# ACDC - http://en.wikipedia.org/?title=AC/DC
#     Australian, awesome and on a highway to hell.

string ROCON_VERSION=acdc

######################## Rocon URIS ###########################################

# These are a convenience to help avoid typos and make easy lookups whatever
# language you are in. The real deal is in rocon_uri/yaml/rules.yaml - make
# sure these stay up to date with respect to each other (TODO: perhaps an
# autogenerator of this msg file?).

###### GENERAL IDENTIFIERS ######

string URI_WILDCARD=*

###### HARDWARE_PLATFORMS ######
string HW_PC=pc
string HW_TURTLEBOT2=turtlebot2
string HW_PR2=pr2
string HW_WAITERBOT=waiterbot
string HW_ROBOT_OTHER=robot_other
# Phones
string HW_GALAXY=galaxy
string HW_MEGA=mega
string HW_NOTE3=note3
string HW_PHONE_OTHER=phone_other
# Tablets
string HW_XOOM=xoom
string HW_NOTE10=note10
string HW_TABLET_OTHER=tablet_other

###### APPLICATION_FRAMEWORKS ######
string APPLICATION_FRAMEWORK_OTHER=application_framework_other
string APPLICATION_FRAMEWORK_OPROS=opros
# Ros
string APPLICATION_FRAMEWORK_GROOVY=groovy
string APPLICATION_FRAMEWORK_HYDRO=hydro
string APPLICATION_FRAMEWORK_INDIGO=indigo
string APPLICATION_FRAMEWORK_ROS_OTHER=ros_other

###### OPERATING SYSTEMS ######
string OS_OSX=osx
string OS_FREEBSD=freebsd
# Windows
string OS_WINXP=winxp
string OS_WINDOWS7=windows7
# linux
string OS_ARCH=arch
string OS_DEBIAN=debian
string OS_FEDORA=fedora
string OS_GENTOO=gentoo
# linux/ubuntu 
string OS_PRECISE=precise
string OS_QUANTAL=quantal
string OS_RARING=raring
string OS_SAUCY=saucy
# Android
string OS_HONEYCOMB=honeycomb
string OS_ICE_CREAM_SANDWICH=ice_cream_sandwich
string OS_JELLYBEAN=jellybean
string OS_KITKAT=kitkat
# Web
string OS_CHROME=chrome
string OS_FIREFOX=firefox
string OS_INTERNET_EXPLORER=internet_explorer
string OS_SAFARI=safari
string OS_OPERA=opera

######################## Rocon Export Tags################################

# Package exports used in concert_service_manager and rocon_app_utilities
string TAG_SERVICE=concert_service
string TAG_RAPP=rocon_app
string TAG_GAZEBO_ROBOT_TYPE=concert_gazebo
string TAG_SOFTWARE=software_farm
