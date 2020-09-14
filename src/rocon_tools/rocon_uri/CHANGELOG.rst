Changelog
=========

0.2.0 (2015-08-18)
------------------
* platform rocon uri generator

0.1.13 (2015-01-12)
-------------------
* Forgotten colon for yaml dic syntax.
* Adding mint rebecca version so the rules
* Contributors: Daniel Stonier

0.1.12 (2015-01-08)
-------------------
* add mint in operating system closes `#69 <https://github.com/robotics-in-concert/rocon_tools/issues/69>`_
* Contributors: Jihoon Lee

0.1.11 (2014-12-02)
-------------------

0.1.10 (2014-11-21)
-------------------
* Merge pull request `#62 <https://github.com/robotics-in-concert/rocon_tools/issues/62>`_ from robotics-in-concert/web
  add web browser as operating system
* add furo as robot
* move web to os
* add web browser as application_framework for webapp launching in web_remocon
* [rocon_uri] helpful pointer to the rocon uri rules.
* Update rules.yaml
* Contributors: Daniel Stonier, Jihoon Lee

0.1.9 (2014-08-25)
------------------
* move from symbolic links to includes for changelogs to avoid eclipse bewilderment.
* debugging error added for catching pid problem, `#53 <https://github.com/robotics-in-concert/rocon_tools/issues/53>`_.
* Contributors: Daniel Stonier

0.1.7 (2014-05-26)
------------------
* add korus to the list of robots.
* make the python module directly responsible for rules.yaml, `#44 <https://github.com/robotics-in-concert/rocon_tools/issues/44>`_
* adds gopher to the list of robots.
* Contributors: Daniel Stonier, Marcus Liebhardt

0.1.5 (2014-05-05)
------------------
* sphinx documentation updated.
* add hardware platform for utexas BWI segbot
* Contributors: Daniel Stonier, Jack O'Quin

0.1.4 (2014-04-16)
------------------
* rocon_uri parse provide all parsed information
* description fix
* Contributors: Daniel Stonier, Jihoon Lee

0.1.3 (2014-04-09)
------------------
* more precise rocon uri os fields for pc interactions, also added trusty to the os rules.
* Contributors: Daniel Stonier

0.1.2 (2014-04-02)
------------------
* a rocon_uri command line tool.
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* sort the elements alphabetically and in reverse so matching doesn't come
  up short as in `#17 <https://github.com/robotics-in-concert/rocon_tools/issues/17>`_.
* adds robots (robosem, turtlebot) and sorts robots alphabetically
* nosetest for checking thrown exceptions on compatibility checking.
* adds robot kobuki to the rules
* rocon uri support for indigo.
* adjustments to drop heir-part of uri if no concert name.
* Revert "switch wildcard to 'any'"
  This reverts commit 3cad70f34bf988c3bce941ddcae301f476fd8519. Using 'any' just looks really awkward and far less instantly recognisable than *.
* switch wildcard to 'any'
* wildcards moved out of the yaml specification
* add android operating systems.
* rocon_uri now loads from yaml.
* first iteration of full rocon uri class and test program.
* ebnf rule parser moved to its own package.
* adding rule parser package and a test.
* Contributors: Daniel Stonier, Marcus Liebhardt
