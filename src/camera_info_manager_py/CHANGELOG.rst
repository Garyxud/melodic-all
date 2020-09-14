Change history
==============

0.2.3 (2014-05-13)
------------------

 * Only use rostest when testing enabled (`#6`_), thanks to Lukas
   Bulwahn.
 * Move repository to ros-perception.

0.2.2 (2013-07-25)
------------------

 * Add namespace parameter to constructor (`#9`_), so a driver can
   handle multiple cameras. Enhancement thanks to Martin Llofriu.
 * Make unit tests conditional on ``CATKIN_ENABLE_TESTING``.
 * Release to Groovy and Hydro.

0.2.1 (2013-04-14)
------------------

 * Set null calibration even when URL invalid (`#7`_).
 * Release to Groovy and Hydro.

0.2.0 (2013-03-28)
------------------

 * Convert to catkin (`#3`_).
 * Remove roslib dependency (`#4`_).
 * Release to Groovy and Hydro.

0.1.0 (2012-12-05)
------------------

 * Initial Python camera_info_manager release to Fuerte.

.. _`#3`: https://github.com/jack-oquin/camera_info_manager_py/issues/3
.. _`#4`: https://github.com/jack-oquin/camera_info_manager_py/issues/4
.. _`#6`: https://github.com/jack-oquin/camera_info_manager_py/pull/6
.. _`#7`: https://github.com/jack-oquin/camera_info_manager_py/issues/7
.. _`#9`: https://github.com/jack-oquin/camera_info_manager_py/pull/9
