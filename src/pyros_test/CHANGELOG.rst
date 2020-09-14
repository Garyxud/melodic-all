^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyros_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2016-11-07)
------------------
* Merge pull request `#5 <https://github.com/asmodehn/pyros-test/issues/5>`_ from asmodehn/string_sub_node
  implementing a simple string_sub_node
* implementing a simple string_sub_node
* now able to name nodes via first command argument.
* Contributors: AlexV, alexv

0.0.5 (2016-10-24)
------------------
* now using params within nodes exposing a service.
  added gitignore.
* adding message type definition for pyros to use optional ros fields.
* installing build-essential in container seems needed by catkin.
* lowering std_msgs requirements. attempt to fix travis issues.
* fixing travis_checks
* now testing with docker. added kinetic.
* Contributors: alexv

0.0.4 (2016-06-02)
------------------
* now travis checking both indigo and jade
* Merge pull request `#2 <https://github.com/asmodehn/pyros-test/issues/2>`_ from asmodehn/package_v2
  Package v2
* README fix
* package description fix
* package v2 wiht minimum required version for dependencies
* Contributors: AlexV, alexv

0.0.3 (2016-02-18)
------------------
* setting version 0.0.3
* replaced an forgotten pyros.srv reference by pyros_test.srv
* Contributors: alexv

0.0.2 (2016-01-10)
------------------
* getting ready for 0.0.2
* adding more nodes from pyros.
* adding nodes from pyros.
* Contributors: AlexV

0.0.1 (2016-01-06)
------------------
* removed unused dependency on python-six since it blocks saucy release.
* removed pyros_setup dependency. should be done outside, before using pyros_test.
* adding travis badge.
* adding travis config.
* renaming and fixing delayed import.
* first commit extracted from pyros
* Contributors: alexv
