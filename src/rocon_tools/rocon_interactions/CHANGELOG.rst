Changelog
=========

0.3.2 (2016-05-11)
------------------
* test case for global executable with args
* longer looping between warning messages when checking for the rapp list

0.3.1 (2016-03-16)
------------------
* [rocon_interactions] bugfix various pairing problems

0.3.0 (2015-10-10)
------------------
* overhauled for better pairing management
* user side can now be dependent on a paired rapp

0.2.0 (2015-08-18)
------------------
* required dependency on a running rapp for interactions
* sleep function exception handling

0.1.18 (2015-05-06)
-------------------
* get_interctions accept empty rocon_uri as wild card closes `#85 <https://github.com/robotics-in-concert/rocon_tools/issues/85>`_
* Contributors: Jihoon Lee

0.1.17 (2015-04-06)
-------------------
* ros param binding to resolve `#81 <https://github.com/robotics-in-concert/rocon_tools/issues/81>`_
* set the defualt values in interactions.js
* Contributors: Jihoon Lee, dwlee

0.1.16 (2015-03-02)
-------------------

0.1.15 (2015-02-27)
-------------------
* [rocon_interactions] android demo finally working.
* Contributors: Daniel Stonier

0.1.14 (2015-02-09)
-------------------
* default icon if resource is not found  closes `#76 <https://github.com/robotics-in-concert/rocon_tools/issues/76>`_
* resource to filepath
* fix wrong code documents
* fix bug
* load fucntion divide up into loading from file and resource part
* update interaction loader for running with absolute path
* Contributors: Jihoon Lee, dwlee

0.1.13 (2015-01-12)
-------------------

0.1.12 (2015-01-08)
-------------------

0.1.11 (2014-12-02)
-------------------
* add webserver address example
* fix invalid compatibility fix `#67 <https://github.com/robotics-in-concert/rocon_tools/issues/67>`_
* webserver address dynamic binding
* Contributors: Jihoon Lee

0.1.10 (2014-11-21)
-------------------
* update web interaction examples
* Contributors: Jihoon Lee

0.1.9 (2014-08-25)
------------------
* Fix timeouts
* move from symbolic links to includes for changelogs to avoid eclipse bewilderment.
* Contributors: Daniel Stonier, kentsommer

0.1.7 (2014-05-26)
------------------
* documentation specifying updated roslaunch/rosrunnable/web app use of parameters and remaps.
* be robust against socket errors with the master, we must keep spinning.
* update rospy publisher queue_size to avoid warning in indigo.
* missing dependency on rocon_app_manager_msgs, fixes `#43 <https://github.com/robotics-in-concert/rocon_tools/issues/43>`_.
* bugfix initialisation of variables to fix pairing stop app problems, `#38 <https://github.com/robotics-in-concert/rocon_tools/issues/38>`_.
* increase timeout for waiting for rapp manager, refs `#37 <https://github.com/robotics-in-concert/rocon_tools/issues/37>`_.
* Contributors: Daniel Stonier, Jihoon Lee, Marcus Liebhardt

0.1.5 (2014-05-05)
------------------
* threadified the interactions manager rapp handler find routine, closes `#37 <https://github.com/robotics-in-concert/rocon_tools/issues/37>`_.
* pairing interactions support, `#34 <https://github.com/robotics-in-concert/rocon_tools/issues/34>`_.
* web_url(), web_app() specifications for web_urls and web_apps, `#33 <https://github.com/robotics-in-concert/rocon_tools/issues/33>`_.
* Contributors: Daniel Stonier, Jack O'Quin

0.1.4 (2014-04-16)
------------------
* bugfix get_interaction callback, return the msg, not the class, `#24 <https://github.com/robotics-in-concert/rocon_tools/issues/24>`_
* Contributors: Daniel Stonier

0.1.3 (2014-04-09)
------------------
* rocon_uri variable name clashes with module name, fixed.
* get_roles moved to a service.
* more precise rocon uri os fields for pc interactions, also added trusty to the os rules.
* catch invalid uri's when filtering.
* bugfix the request interactions hash handling, had legacy msg code still being used.
* Contributors: Daniel Stonier

0.1.1 (2014-04-01)
------------------
* test dependencies.
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* documentation
* roslint for rocon_interactions
* tutorials
* listener html example
* javascript library to parse interaction queries for web apps.
* yaml structures instead of strings for parameters in interaction yamls
* interactions for web apps
* publish roles after pre-loading, fixes `#7 <https://github.com/robotics-in-concert/rocon_tools/issues/7>`_.
* a test with fake remocons
* symbol binding for rosbridge parameters, `#6 <https://github.com/robotics-in-concert/rocon_tools/issues/6>`_
* rosbridge support for the interaction manager
* Contributors: Daniel Stonier, Jihoon Lee
