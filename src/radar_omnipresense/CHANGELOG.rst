^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package radar_omnipresense
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2018-09-10)
------------------
* Remove use of rapidjson
* make compatible with OPS241 (API v 1.0-1-1) and OPS242 (v1.2 and beyond)
* Contributors: Jim Whitfield

0.2.0 (2018-05-31)
------------------
* serialconnection folder
* added rapidjson, which has all needed header files
* added rapidjson folder in lib folder
* added rapidjson include the lib folder and 'wrapped' it in a radar_omnipresense namespace
* Contributors: Garren Hendricks, Jim Whitfield

0.1.0 (2018-05-11)
------------------
* commiting the 2011 api version of the code. The branch RapidJSON_preserve contians 2015 api version. That branch should not be modified at all
* altered findrapidjson
* updated CMakeLists and utest to 'pass'
* adding files for very simple 'unit testing'
* Updated the readme to no longer say that you need to download and install LinuxCommConnection for the package
* Added the lib file so that linuxcommconnection is no longer a depedency issue
* Prepping for ros package submittal
* Added the lib file so that linuxcommconnection is no longer a depedency issue
* address RapidJSON dependency
* added raw msgs
* Contributors: Garren Hendricks, Jim Whitfield 

0.0.1 (2018-04-07)
------------------
* Merge branch 'master' of https://github.com/SCU-RSL-ROS/radar_omnipresense
* added doxygen detectable documentation for the function prototypes
* changed the topic name from 'radar' to 'radar_report'
* sensor id field shows the serial port that the radar is connected to
* added the rosluanch file for a single radar sensor and then have a rosluanch file that will launch multiple radar sensors, currently it launches 2
* Added unix time stamp to msg data
* Adding folder udev to contain device rules for usb radar device. also adding a bash script to move the rules to proper filesystem location.
* Create LICENSE
* added Findrapidjson.cmake
* Contributors: Garren Hendricks, Matthew Condino, Pillager225, RyanLoringCooper, Jim Whitfield
