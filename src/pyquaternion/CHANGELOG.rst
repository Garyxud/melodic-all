^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyquaternion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.6 (2020-02-10)
------------------
Post ROS package conversion

* Change exec depend naming from numpy to python-numpy
* Add numpy dependency to package.xml
* Create catkin package, rename and move some files
* Fix casting error in trace_method
* Add setter for vector

0.9.5 (2019-02-10)
------------------
Pre ROS package conversion

* Merge pull request #49 from KieranWynn/handedness-clarification
  reference frame clarification
* Replac kwargs with atol and rtol explicitly
* Updated index.md with examples of how to use new options
* Update deploy config
* Merge pull request #46 from e-kwsm/special-methods
  Add special methods: __abs_\_; __matmul_\_, __imatmul_\_ and __rmatmul\_\_
* minor fixes
* Quaternion from Matrix Optional Args
* Merge pull request #43 from Hojjatrt/master
