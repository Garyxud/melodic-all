^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nerian_stereo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.8.0 (2020-07-29)
------------------
* Updated to nerian software release version 8.1.0
* Support for Support for 1 to 3 images in result set
* Added new device configuration variables
* Contributors: Konstantin Schauwecker, Ramin Yaghoubzadeh Torky

3.7.0 (2020-02-21)
------------------
* Added new trigger configuration parameters
* Updated to vision software release 7.2
* Contributors: Konstantin Schauwecker

3.6.0 (2019-12-05)
------------------
* Updated Nerian software release to version 7.1.0
* Correct pixel format for publication of RGB camera images
* Fixed problem with LD_LIBRARY_PATH when running from catkin workspace
* Contributors: Konstantin Schauwecker, Ramin Yaghoubzadeh Torky

3.5.0 (2019-08-15)
------------------
* Updated to libvisiontransfer 7.0.0
* Contributors: Konstantin Schauwecker

3.4.0 (2019-06-19)
------------------
* Updated to latest Nerian software release 6.5.0
* New nodelet version of ROS module
* Support for dynamic_reconfigure
* Corrected marking of point clouds as not dense

3.3.2 (2019-02-15)
------------------
* Fixed installation permissions
* Contributors: Konstantin Schauwecker

3.3.1 (2019-02-08)
------------------
* Compile fix for older Ubuntu versions

3.3.0 (2019-02-04)
------------------
* Updated to Nerian vision software release 6.4.0
* Added functionality to use timestamps from SceneScan for timestamping transmitted messages.
* Contributors: Konstantin Schauwecker

3.2.1 (2018-12-05)
------------------
* Updated Nerian vision software to version 6.2.1
* Added curl dependency to package.xml
* Contributors: Konstantin Schauwecker

3.2.0 (2018-11-27)
------------------
* Added support for colored point clouds
* Automated detection of libvisiontransfer version
* Updated nerian vision software to 6.2.0
* Contributors: Konstantin Schauwecker

3.1.1 (2018-11-13)
------------------
* Fixed CMake build problem
* Contributors: Konstantin Schauwecker

3.1.0 (2018-11-10)
------------------
* Removed arch=native flag from compiler options
* Updated nerian vision software to 6.1.1
* Added support for combined RGB color channel in pointcloud
* Contributors: Konstantin Schauwecker

3.0.2 (2018-08-07)
------------------
* Fixed build errors

3.0.1 (2018-08-06)
------------------
* Installing missing scripts for binary release
* Contributors: Konstantin Schauwecker

3.0.0 (2018-06-08)
------------------
* Updated to Nerian software release 6.0.0
* Contributors: Konstantin Schauwecker

2.2.0 (2018-05-13)
------------------
* Added support for RGB point cloud output
* Contributors: Konstantin Schauwecker

2.1.0 (2017-12-09)
------------------
* New color coding scheme: rainbow
* Automatic selection of color legend range
* Contributors: Konstantin Schauwecker

2.0.3 (2017-10-20)
------------------
* Build fix for Ubuntu Zesty
* Contributors: Konstantin Schauwecker

2.0.2 (2017-10-18)
------------------
* Updated to libvisiontransfer 5.0.1, which fixes synchronization bug
* Contributors: Konstantin Schauwecker

2.0.1 (2017-10-02)
------------------
* Fixed support for 12-bit images
* Fixed build problems

2.0.0 (2017-09-29)
------------------
* Renamed node to nerian_stereo
* Updated to libvisiontransfer 5.0.0 to support new SceneScan sensor
* Contributors: Konstantin Schauwecker

1.6.2 (2017-05-30)
------------------
* Allow launch even if calibration file is not found
* Implemented upper limit for point cloud depth (max_depth parameter)
* Contributors: Konstantin Schauwecker

1.6.1 (2017-03-27)
------------------
* Updated libvisiontransfer to version 4.1.2
* Contributors: Konstantin Schauwecker

1.6.0 (2017-02-15)
------------------
* Updated SP1 software to version 4.1.0
* Script and launch file for downloading camera calibration
* Added optional execution delay
* Contributors: Konstantin Schauwecker

1.5.1 (2017-01-19)
------------------
* Added proper error reporting in case of exceptions
* Contributors: Konstantin Schauwecker

1.5.0 (2017-01-17)
------------------
* Switched to new sp1 software release 4.0.0
* Added example code for operation mode configuration to launch script
* Added example scripts for switching SP1 operation mode
* Separate topic for right image and bugfix for right image output
* Contributors: Konstantin Schauwecker

1.4.0 (2016-10-07)
------------------
* Updated to SP1 software release 3.0.0
* Removed automatic installation of spcom
* Handling of point cloud exceptions
* Contributors: Konstantin Schauwecker

1.3.3 (2016-05-17)
------------------
* Updated SP1 software release to version 2.1.6
* Contributors: Konstantin Schauwecker

1.3.2 (2016-05-09)
------------------
* Build fix for ROS kinetic
* Contributors: Konstantin Schauwecker

1.3.1 (2016-05-05)
------------------
* Added missing launch file to ROS package
* Contributors: Konstantin Schauwecker

1.3.0 (2016-03-18)
------------------
* Updated sp1 software release to version 2.1.5
* Support for changing q-matrix (caused by auto re-calibration)
* Contributors: Konstantin Schauwecker

1.2.2 (2016-02-12)
------------------
* Upgraded libvisiontransfer to version 2.1.2
* Contributors: Konstantin Schauwecker

1.2.1 (2016-01-12)
------------------
* Upgraded libvisiontransfer to version 2.1.1
* Contributors: Konstantin Schauwecker

1.2.0 (2015-11-23)
------------------
* Added current release candidate of libvisiontransfer 2.0.0
* Adaptations for libvisiontransfer 2.0.0
* Support transfer of Q matrix
* Contributors: Konstantin Schauwecker

1.1.2 (2015-10-05)
------------------
* Fixed bug that prevented conversion of point cloud message to PCL object
* Contributors: Konstantin Schauwecker

1.1.1 (2015-09-15)
------------------
* Updated to libvisiontransfer 1.0.2
* Installing libvisiontransfer headers
* Contributors: Konstantin Schauwecker

1.1.0 (2015-08-26)
------------------
* Cleaned-up example launch file
* Minor bugfixes
* Updated SP1 software package
* Publishing of camera information
* Optional disparity window
* Performance optimization
* Removed enable parameters
* Fixed ROS coordinate system
* Contributors: Konstantin Schauwecker

1.0.2 (2015-08-25)
------------------
* Minor fixes to build files
* Contributors: Konstantin Schauwecker

1.0.1 (2015-08-25)
------------------
* Initial release
* Contributors: Konstantin Schauwecker, nerian-vision
