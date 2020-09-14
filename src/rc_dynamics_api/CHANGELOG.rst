0.10.0 (2019-09-11)
------------------

* improve exception handling using new NotAvailable

0.9.0 (2019-09-11)
------------------

* add getCam2ImuTransform

0.8.0 (2019-05-08)
------------------

* BREAKING CHANGE: major refactoring to comply to naming conventions
* improve cleanup of requested streams
* add support for http status code 429: too many requests
* improve error handling and messages
* add state getters for rc_dynamics, rc_slam, rc_stereo_ins
* return UNKNOWN if dynamics state is not accessible
* improve startup behaviour of RemoteInterface

0.7.1 (2019-02-04)
------------------

* update cmake files for version and packaging

0.7.0 (2018-07-02)
------------------

* add saveSlamMap, loadSlamMap and removeSlamMap methods
* getSlamTrajectory doesn't time out by default anymore

0.6.0 (2018-04-19)
------------------

* add methods to access slam "reset" service
* use single json.hpp include instead of submodule

0.5.0 (2018-02-26)
------------------

* Updates for rc_visard image version v1.1.x with support for SLAM
* added startSlam, restartSlam, stopSlam and getTrajectory
* support Windows build

0.4.0 (2017-09-27)
------------------

* refactoring: simplified csv-printing in rcdynamics_stream tool
* refactoring: better exception handling
* refactoring: renaming, wording is now PbMsgType instead of ProtoType
* added '-l option' to rcdynamics_stream tool: list avail. streams
* improvements and bug fixes in cmake
* use rc_dynamics_msgs submodule


0.3.0 (2017-09-06)
------------------

* start/stop rc_dynamics node instead of rc_stereo_ins
* don't start/stop rc_dynamics for imu stream

0.2.1 (2017-08-31)
------------------

* bug fix: header not installed

0.2.0 (2017-08-30)
------------------

* bug fix: wrong name of tool: vins_stream -> rcdynamics_stream

0.1.0 (2017-08-29)
------------------

* Initial release
