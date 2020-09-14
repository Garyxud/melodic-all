^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ixblue_stdbin_decoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2020-08-31)
------------------
* Fix CMake config file for older CMake
  In order to do a Kinetic release.
* Contributors: Romain Reignier

0.1.1 (2020-08-27)
------------------
* Reduce minimum required CMake version to allow build on Debian Stretch for ROS Melodic
* Contributors: Romain Reignier

0.1.0 (2020-08-26)
------------------
* Add package.xml to build with catkin
* Rework cmake installation
* Rename to ixblue_stdbin_decoder in lower case to stay consistent
* Allow the parser to read other header types (Answer and command)
* Use boost::asio::const_buffer instead of boost::asio::mutable_buffer
  Frame data is not modified while parsed.
* Change namespace to ixblue_stdbin_decoder to avoid naming conflict with StdBinDecoder class
  Causes issues in IDEs and adds confusion
* Contributors: Adrien BARRAL, Romain Reignier
