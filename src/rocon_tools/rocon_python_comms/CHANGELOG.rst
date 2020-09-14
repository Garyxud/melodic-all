Changelog
=========

0.3.2 (2016-05-11)
------------------
* master check via the rosparam server bugfix
* allow named services in batching functions
* drop pyros_test

0.3.1 (2016-03-16)
------------------
* [rocon_python_comms] connection cache node

0.3.0 (2015-10-10)
------------------
* lots of minor api added

0.2.0 (2015-08-18)
------------------
* one shot and indefinite waiting for service pair clients
* basename function for ros names
* convenience service/publisher/subscriber mass producers
* connection cache node prototype
* connection cache class moved here from the gateway for multi-use

0.1.16 (2015-03-02)
-------------------
* use exceptions informative way `#79 <https://github.com/robotics-in-concert/rocon_tools/issues/79>`_
* update logic of find_service_namespace `#79 <https://github.com/robotics-in-concert/rocon_tools/issues/79>`_
* rename is_valid_service
* rename is_valid_service to service_is_available
* update namespace finder and service validation checker
* Contributors: Jihoon Lee, dwlee

0.1.15 (2015-02-27)
-------------------

0.1.14 (2015-02-09)
-------------------

0.1.13 (2015-01-12)
-------------------
* delete command option about port and implement to set port when terminal is spawned
* make remocon_delay test
* Contributors: dwlee

0.1.12 (2015-01-08)
-------------------

0.1.11 (2014-12-02)
-------------------

0.1.10 (2014-11-21)
-------------------

0.1.9 (2014-08-25)
------------------
* to fix `#54 <https://github.com/robotics-in-concert/rocon_tools/issues/54>`_
* move from symbolic links to includes for changelogs to avoid eclipse bewilderment.
* Contributors: Daniel Stonier, Jihoon Lee

0.1.7 (2014-05-26)
------------------
* update publisher queue_size to avoid warning in indigo.
* robustify find_service against zombie services, closes `#42 <https://github.com/robotics-in-concert/rocon_tools/issues/42>`_.
* updated usage examples for service pairs.
* Contributors: Daniel Stonier

0.1.5 (2014-05-05)
------------------
* sphinx documentation added.
* better feedback on service communication errors.
* comment find_node that it only accepts unresolved names.
* fix missing list return for non-unique find_service requests
* Contributors: Daniel Stonier

0.1.2 (2014-04-02)
------------------
* bugfix location of rospair bash file.
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* bugfix missing list return for non-unique find_topic_requests.
* add find_node function including rostests.
* add find_topic function.
* find_service should catch exceptions when ros shuts down too.
* add find_service function.
* optional threaded callbacks for service pair servers.
* testing against quick calls of the service server.
* wall rate from rocon_utilities.
* subscriber proxy from rocon_utilities.
* rospair command line tool with call, type, list and bash completion.
* rospair command line tool list functionality
* ported in service pairs from multimaster.
* Contributors: Daniel Stonier, Marcus Liebhardt
