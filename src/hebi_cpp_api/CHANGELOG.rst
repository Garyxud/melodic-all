^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hebi_cpp_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2020-4-3)
----------------
* added experimental high-level "Arm API" to enable easier control of robotic arm systems

3.1.1 (2019-12-16)
------------------
* Fix incorrect behavior when getting and setting IO pin values

3.1.0 (2019-12-02)
------------------
* Reduce conversion needs by adding (deprecated) overloads for:

  * getJ
  * getJacobians
  * getFK
  * getForwardKinematics
  * getFrameCount

* Fix multiple definition error

(from 3.0.0)

* Robot Model:

  * Added "input frame" type for forward kinematics operations
  * Added end effector support (custom and parallel gripper types)
  * Added R-series support (actuator, link, bracket, and end effector)
  * Added options for link input + output type
  * Support import of HRDF format 1.2.0

* Robot Model:

  * removed "combine" functionality for addJoint and addRigidBody
  * now only allows addition of elements which match the physical interface of the previous element
  * changed the behavior of "end effector" frames; by default, none are returned any unless an "end effector" is specifically added
  * Changed usages of HebiJointType, HebiFrameType, and HebiRobotModelElementType C-style enums to C++ scoped enums

* Fixed bug when setting IO pins in commands; commands would sometimes affect other pins.

(from 2.2.0)

* Added ability to set and clear text in the experimental mobile IO API
* Added ability to get raw feedback from experimental mobile IO API

2.1.0 (2019-08-21)
------------------
* Updated various messages:

  * Info:

    * Added "serial" getter for Info packets

  * Info and Command:

    * Added mstop strategy
    * Added position limit strategies
    * Added velocity limits
    * Added effort limits
    * Added flag for whether or not accelerometer feedback includes gravity (on supporting devices, namely Mobile IO devices

  * Command:

    * Added ability to set strings for and clear the "log" text field in the Mobile IO apps 

  * Feedback:

    * Added "pwm command" feedback

* Add "robot element metadata" that allows for introspection of RobotModel objects.
* Import/Export safety parameters from/to a file into/from GroupCommand objects
* Export safety parameters to a file from GroupInfo objects
* Added "experimental" namespace intended for feature-preview items
* Added "mobile io wrapper" to experimental namespace that allows for easier interface with Mobile IO devices 
* Update core C API from 1.4.2 to 1.8.0

  * Significantly faster Jacobian computation
  * Full wildcard lookup supported when creating groups
  * Significantly faster trajectory solver implementation
  * Added "subaddress" support in lookup, commands, feedback, and logging; allows for simulator support

* Cleaned up code style:

  * default destructors and accessibility for deleted copy/move assignment operators
  * const on move operators (src/util.hpp)
  * made several getters inline

* Added "FunctionCallResult" used when importing safety parameter files to allow error message to be accessed
* Update core C API from 1.4.2 to 1.8.0

  * Fixed getters for motor position, ar position, ar orientation, ar quality, and battery level in feedback
  * Locale invariant conversion when reading in .xml files, such as gains and HRDF (always expect "1.23" instead of "1,23", regardless of system's locale setting)
  * Use Ethernet header instead of message packet content to discover modules on the network (fixes issue when using multiple interfaces - wired and wireless - on iPad or Android running HEBI Mobile I/O)

2.0.2 (2019-01-29)
------------------
* Make package installable
* Moved the header files into an include directory
* Removed the Eigen folder; use ROS package instead
* Fixed CMake for installable package

  * Addressed Eigen dependency
  * Installed include files and libraries correctly

* NOTE: this does not correspond with an official 2.0.2
  release of the upstream HEBI C++ API, because these
  changes were all local ROS build system changes. This
  mismatch will be resolved in v2.1.0.
* Contributors: Matthew Tesch, iamtesch

2.0.1 (2018-12-19)
------------------
* Initial import of the HEBI C++ API v2.0.1

  * Note: package.xml and CMakeLists.txt have been changed to be catkin
    compliant.

* Addressed i386/armhf/aarch64 ros buildfarm issues.
* Contributors: Matthew Tesch
