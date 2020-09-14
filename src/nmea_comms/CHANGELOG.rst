^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nmea_comms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-08-02)
------------------
* Fixed boost::thread_resource_error: Resource temporarily unavailable exception that was thrown using netcat to feed socket_node.cpp with NMEA sentences. (`#5 <https://github.com/ros-drivers/nmea_comms/issues/5>`_)
* Contributors: Avio, Mike Purvis

1.1.0 (2015-04-23)
------------------

* Release to Jade.

1.0.1 (2014-04-21)
------------------
* Add roslint, automatic style fixes.
* Add license headers to source files.
* Tidy up package.xml, add author and url elements.
* Contributors: Mike Purvis

1.0.0 (2014-04-21)
------------------
* Update the tee example launch file.
* Change topic names to match the default in nmea_navsat_driver.
* Put the common sources in a shared lib.
* Add some basic tests.
* Contributors: Mike Purvis

0.0.3 (2013-10-03)
------------------
* Add sleep() call between poll and read to further reduce CPU use.
* Properly apply the specified baud rate.
* Gracefully handle nulls received on the serial line.

0.0.2 (2013-09-23)
------------------
* Add support for setting frame_id on rx Sentence messages.

0.0.1 (2013-09-03)
------------------
* Initial release of bidirectional socket and serial nodes.
