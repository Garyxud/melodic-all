^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2020-06-09)
------------------
* Minor: catkin_lint.
* Fix: prevent deadlocks on stopping timers.
* Fix: Ignore stale frames in subtree search.
* Fix: Fix nodelet namespace.
* Contributors: Martin Pecka

1.0.5 (2020-02-09)
------------------
* Added support for streams whose TF tree can be updated during runtime.
* Added support for initial streams.
* Added possibility to specify desired names of the published transform streams.
* Contributors: Martin Pecka

1.0.4 (2020-02-04)
------------------
* Added a parameter that can help distinguishing between the original and the upgraded TF2 server.
* Added support for running the server as nodelet.
* Added support for updating the requested subtree during runtime.
* Contributors: Martin Pecka

1.0.3 (2020-01-21)
------------------
* Create LICENSE
* Contributors: Martin Pecka

1.0.2 (2020-01-10)
------------------
* Added url tags to package.xml
* Contributors: Martin Pecka

1.0.1 (2020-01-10)
------------------
* Initial version
* Contributors: Martin Pecka
