Changelog
=========

0.1.18 (2015-05-06)
-------------------

0.1.17 (2015-04-06)
-------------------

0.1.16 (2015-03-02)
-------------------

0.1.15 (2015-02-27)
-------------------

0.1.14 (2015-02-09)
-------------------

0.1.13 (2015-01-12)
-------------------
* delete command option about port and implement to set port when terminal is spawned
* update test function for test about without port option in roslauch
* Contributors: dwlee

0.1.12 (2015-01-08)
-------------------

0.1.11 (2014-12-02)
-------------------

0.1.10 (2014-11-21)
-------------------

0.1.9 (2014-08-25)
------------------
* move from symbolic links to includes for changelogs to avoid eclipse bewilderment.
* debugging error added for catching pid problem, `#53 <https://github.com/robotics-in-concert/rocon_tools/issues/53>`_.
* added a bypass and logged a warning when the parent pid is not yet available when cancelling spawned windows, `#53 <https://github.com/robotics-in-concert/rocon_tools/issues/53>`_
* catch an error if a shutdown signal can't find its terminal process.
* Contributors: Daniel Stonier

0.1.8 (2014-05-26)
------------------
* bugfix a variable name typo in ``RosLaunchConfiguration`` (``option``->``options``).

0.1.7 (2014-05-26)
------------------
* expose a public api for other modules (rocon_remocon roslaunchers, turtlesim and gazebo spawners) to use.
* roslaunch configurations get namespace support (i.e. pushing down).
* rocon_launch default port behaviour changed, now uses the ros master env. variable with 11311 as a fallback.
* allow standalone roslaunches in the rocon_launch xml specification `#47 <https://github.com/robotics-in-concert/rocon_tools/issues/47>`_.
* Contributors: Daniel Stonier

0.1.5 (2014-05-05)
------------------
* sphinx documentation added.
* Contributors: Daniel Stonier

0.1.3 (2014-04-09)
------------------
* default arg for when higher level users don't need it.
* Contributors: Daniel Stonier

0.1.1 (2014-04-01)
------------------
* kill the process group when terminating, not the process itself.
* expose get_roslaunch_pids method publicly
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* rocon_launch moved into rocon_tools.
* Contributors: Daniel Stonier
