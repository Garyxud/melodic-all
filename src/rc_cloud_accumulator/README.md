rc_cloud_accumulator
====================

This project demonstrates how to create a registered point cloud map
using the roboception rc_visard with the [ROS driver](http://wiki.ros.org/rc_visard_driver).


What it does
------------
The rc_cloud_accumulator ROS node subscribes to the following topics of the *rc_visard_driver*

 - `/stereo/points2`
 - `/pose`
 - `/trajectory`

The received information is stored, such that a registered point cloud can be
computed and saved to disk. For performance reasons, the point clouds are preprocessed.
See *Filters* below for more details.

The node displays point clouds received on the first topic using the live pose (/pose)
to position them in the global coordinate frame.

The trajectory topic can be used to feed an optimized trajectory from the SLAM module
into the *rc_cloud_accumulator*. The easiest way is to start the *rc_visard_driver*
with the parameter *autopublish_trajectory* set to `True` and call the service 
*/rc_visard_driver/get_trajectory*. The *rc_visard_driver* will then send the
trajectory on */trajectory*.

Building
--------

Compilation follows the standard build process of ROS. You can also do the regular cmake cycle:

- `source /opt/ros/`*your-ROS-Distro*`/setup.bash`
- `mkdir build`
- `cd build`
- `cmake ..`
- `make -j3`

To build a debian package replace the last two steps with

- `cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}" -DCMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}" -DCMAKE_BUILD_TYPE=Release ..`
- `make -j3 package`
- Install the package with `sudo dpkg -i rc_cloud_accumulator*.deb`

Running
-------

After starting the *rc_visard_driver*, execute

`rosrun rc_cloud_accumulator rc_cloud_accumulator`

Example with parameter:

`rosrun rc_cloud_accumulator rc_cloud_accumulator _voxel_grid_size_display:=0.01`

Known Bugs
----------

- You have to change the view (mouse drag or mouse wheel) before the clouds are displayed
- On Ubuntu trusty, the window can only be closed terminating the console application (e.g., Ctrl-C).
  Probably fixed (see [this issue](https://github.com/PointCloudLibrary/pcl/issues/172)) on newer versions.

Point Cloud Filters
-------

For performance reasons the point clouds are by default filtered in several stages.
The filters are parameterized via ROS parameters.
All filters can be turned off using appropriate settings.
See *ROS Parameters* for a detailed description of the parameters.

When a point cloud is received, the points will first be filtered by
a minimum and maximum distance along the optical axis.

A copy of the resulting cloud will be stored in memory for later use.

The point cloud will then be transformed to the global coordinate
frame and merged into the currently displayed point cloud. To keep the
visualization snappy, the displayed point cloud will be filtered with a voxel
grid using the parameter *voxel_grid_size_display*.

When the save_cloud service (see below) is used, the stored point clouds will
be merged (considering pose updates received in the meantime). The result will
also be be filtered with a voxel grid using the parameter
*voxel_grid_size_save*.


ROS Services
------------

The *rc_cloud_accumulator* provides the following services

 - `/rc_cloud_accumulator/toggle_pause`: Toggle processing of received data
 - `/rc_cloud_accumulator/save_cloud`: Register stored clouds and save them to disk.
   The stored cloud will be displayed, but updated with the next incoming point cloud.
   To keep the display of the stored result pause before saving.

ROS Parameters
--------------

  - `voxel_grid_size_display` (default = 0.05m): Downsampling grid size of the point cloud in the live display. Set to zero or below to turn off.
  - `voxel_grid_size_save` (default = 0.01m): Downsampling grid size of the point cloud when saving to disk. Set to zero or below to turn off.
  - `minimum_distance` (default =  0.0m): Omit points closer to the rc_visard
  - `maximum_distance` (default = 10.0m): Omit points closer farther from the rc_visard. Set below minimum to turn distance filtering off.
  - `output_filename` (default = "cloud.pcd")
  - `start_paused` (default = false)
  - `keep_high_resolution` (default = true): Set to false to save memory and processing time.
    The original point clouds will not be stored and the point cloud saved to
    disk will be the one that is displayed. Only the voxel grid filter for the
    *live* pose will be applied. Also, correction of the point cloud poses via
    the SLAM trajectory is not possible in this mode.


