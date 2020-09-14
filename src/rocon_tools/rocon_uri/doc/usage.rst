Usage
=====

In general, store, pass around and manipulate rocon uri's as strings. Create
:class:`rocon_uri.uri.RoconURI` objects only as needed when you need to identify
individual elements of the uri or or you need to do comparisons.
This is a convention generally followed by urllib as well and is easiest in practice.

Parsing w/ Python Module
------------------------

To create/validate a rocon uri object, simply pass in the rocon uri string(s) to the parser and
access via the specialised descriptors:

.. code-block:: python

   try:
       rocon_uri_object = rocon_uri.parse('rocon:/turtlebot2|pr2/dude/hydro/precise#rocon_apps/chirp')
       print rocon_uri_object.hardware_platform.list       # output: ['turtlebot2', 'pr2']
       print rocon_uri_object.hardware_platform.string     # output: 'turtlebot2|pr2'
       print rocon_uri_object.name.string                  # output: 'dude'
       print rocon_uri_object.application_framework.string # output: 'hydro'
       print rocon_uri_object.operating_system.string      # output: 'precise'
       print rocon_uri_object.rapp                         # output: 'rocon_apps/chirp'
   except rocon_uri.RoconURIValueError as e:
       print("Invalid rocon uri string [%s]" % str(e))

Note the calls to the hardware_platform field via the list and string accessors. This is the same for
any of the primary fields stored by a :class:`RoconURI <rocon_uri.uri.RoconURI>` object

* hardware_platform
* name
* application_framework
* operating_system

The fragment part (rapp) does not follow the same rule and can be more simply accessed directly as shown.

Compatibility Testing w/ Python Module
--------------------------------------

Compatibility between two rocon uri strings is often used to determine if an application is runnable on
a particular platform, or to compare whether a particular resource (e.g. robot) is able to satisfy
a resource request from a rocon service:

.. code-block:: python

   try:
       rocon_uri_string_a = 'rocon:/turtlebot2/dude/hydro/precise'
       rocon_uri_string_b = 'rocon:/turtlebot2/*/hydro/precise'
       compatible = rocon_uri.is_compatible(rocon_uri_string_a, rocon_uri_string_b)
   except rocon_uri.RoconURIValueError as e:
       print("Invalid rocon uri string(s) passed [%s]" % str(e))

Command Line Tools
------------------

There is a *rocon_uri* command line tool which can help introspect rocon uri strings while offline.::

    > rocon_uri help
    
    Utility for introspecting on rocon uri strings.
    
    Commands:
            rocon_uri parse URI     parse and attempt to validate a rocon URI.
            rocon_uri fields        print a full list of permitted fields in a rocon uri string.
            rocon_uri rules         print a full list of the ebnf rules for a rocon uri string.

Unit Test Examples
------------------

A more complete list of examples which may be useful to refer to can be found in the exhaustive list of
unit test cases:

.. literalinclude:: ../tests/test_rocon_uri.py