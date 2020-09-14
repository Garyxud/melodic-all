Changelog
=========

0.7.12 (2015-07-09)
-------------------

0.7.11 (2015-05-27)
-------------------

0.7.10 (2015-04-06)
-------------------

0.7.9 (2015-02-09)
------------------

0.7.8 (2014-11-21)
------------------

0.7.6 (2014-08-25)
------------------
* Export architecture_independent flag in package.xml
* add public parameters in resource
* run_depend 'message_runtime' is not listed in catkin_package()
* Contributors: Jihoon Lee, Scott K Logan

0.7.2 (2014-04-16)
------------------
* extra status flag for resources that have cleanly left the concert.
* Contributors: Daniel Stonier

0.7.1 (2014-04-09)
------------------
* updated invalid error code comment
* convenience string for introspecting on the reason errors.
* changelog entries fixed.
* Contributors: Daniel Stonier

0.7.0 (2014-03-29)
------------------
* targeted for Indigo Igloo release.

0.6.6 (2014-04-29)
------------------
* Revise status labels, removing old ones (`#60`_):
    - ABORTED -> CLOSED
    - REJECTED -> CLOSED
    - RELEASING -> CANCELING
    - RELEASED -> CLOSED
* Add ``reason`` field with associated labels (`#60`_):
    - NONE: no reason provided
    - PREEMPTED: preempted for higher-priority task
    - BUSY: requested resource busy elsewhere
    - UNAVAILABLE: requested resource not available
    - TIMEOUT: lost contact with requester
* Define more priority labels:
    - BACKGROUND_PRIORITY: when nothing else to do
    - LOW_PRIORITY: low-priority task
    - DEFAULT_PRIORITY: sane default priority
    - HIGH_PRIORITY: high-priority task
    - CRITICAL_PRIORITY: mission-critical task

0.6.5 (2013-12-19)
------------------
* Initial version of an experimental scheduler message package for
   Hydro (`#27`_).
* Updates due to experience with prototype implementation (`#41`_):
    - add new SchedulerRequests message type
    - deprecate AllocateResources and SchedulerFeedback
    - add new RESERVED status to Request message
    - add hold_time duration to Request message
* Add priority to Request message (`#43`_).
* Removed unneeded dependency on ``concert_msgs``.

.. _`#27`: https://github.com/robotics-in-concert/rocon_msgs/pull/27
.. _`#41`: https://github.com/robotics-in-concert/rocon_msgs/issues/41
.. _`#43`: https://github.com/robotics-in-concert/rocon_msgs/issues/43
.. _`#60`: https://github.com/robotics-in-concert/rocon_msgs/issues/60
