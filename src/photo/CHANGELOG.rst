^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package photo
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2019-04-30)
------------------
* Fix /get-config problem with TEXT type (`#8 <https://github.com/bosch-ros-pkg/photo/issues/8>`_)
  Fix a problem where, when trying to call the /get-config
  service on a text field, you would first get wrong data,
  and on the second call the node would crash.
* Fix release (`#11 <https://github.com/bosch-ros-pkg/photo/issues/11>`_)
  * add opencv as dependency
  * style cleanup
* Merge pull request `#6 <https://github.com/bosch-ros-pkg/photo/issues/6>`_ from bosch-ros-pkg/Karsten1987-patch-1
  Make Karsten maintainer
* Make Karsten maintainer
* Fixed error in CMakeLists (`#5 <https://github.com/bosch-ros-pkg/photo/issues/5>`_)
* Contributors: Babis Boloudakis, Karsten Knese, Philip Roan

1.0.2 (2019-03-13)
------------------
* update rosdep keys for libgphoto2
* Contributors: root

1.0.1 (2019-03-13)
------------------
* Melodic devel (`#4 <https://github.com/bosch-ros-pkg/photo/issues/4>`_)
  * Make it compile on Ubuntu Bionic
  Changes from code-iai fork: https://github.com/code-iai/iai_photo
  * Fix delete on array
  * Fix set config for toggle
  A bool was implicitly casted to int through a void*
  * Fix whitespaces
  * Fix segmentation fault if no camera connected
  * Pass std::string by const reference to avoid copies
  * Sanitize a bit more
* Merge pull request `#3 <https://github.com/bosch-ros-pkg/photo/issues/3>`_ from proan/hydro-devel
  Added another dependency and removed superfluous comments.
* Added another dependency and removed superfluous comments.
* Merge pull request `#2 <https://github.com/bosch-ros-pkg/photo/issues/2>`_ from proan/hydro-devel
  Catkin-ized package for photo library.
* Add dependency on message and service generation being complete first.
* Catkin-ized package for photo. This package has been moved from the larger bosch_drivers repository.
* Initial commit
* Contributors: Philip Roan, Romain Reignier
