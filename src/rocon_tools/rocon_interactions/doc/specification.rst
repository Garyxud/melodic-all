Interaction Specification
=========================

Interactions are specified in yaml_ to be loaded onto the interactions manager node,
via either the parameter server or over the node ros api. 

Interaction Types
-----------------

 * Direct executable
 * Rosrun executable
 * Roslaunch launcher
 * Web url
 * Web app
 * Android activity
 
Yaml Specification
------------------

A single interaction specification in yaml has the following format:

.. code-block:: yaml

   name:
   role:
   compatibility:
   display_name:
   description:
   icon:
     resource_name:
   parameters: 
   remappings:
     - remap_from:
       remap_to:

The names above resolve as follows:

 * ``name`` : the name used to determine interaction type and start the interactivity

   * direct executable : either program name (must be on the PATH), or absolute path to the program, e.g. rqt_graph 
   * rosrunnable : a ros resource name (pkg/filename) pair, e.g. rocon_interactions/loader 
   * roslaunchable : a ros resource name (pkg/launcher) where the filename is a .launch file, e.g. rocon_interactions/rviz_markers.launch 
   * web url : a normal web url with the ``web_url()`` wrapper, e.g. web_url(http://wiki.ros.org/rocon_interactions)
   * web app : a normal web url pointing to a valid ros web app with the ``web_app()`` wrapper, e.g. web_app(http://robotics-in-concert.github.io/rocon_interactions/js/listener.html)
   * android activity : the fully qualified name for launching an activity, e.g. com.github.robotics_in_concert.rocon_android.SolutionManager

 * ``role``: the role to embed this interaction in, e.g. admin, dev, customer
 * ``compatibility``: a rocon_uri_ field establishing its platform compatibility
 * ``display_name``: a human friendly name to show to the user
 * ``description``: provide some info about the interaction, important as the name can be usually configured in various ways
 * ``icon``: an icon to represent this interaction (optional)
 
   * ``resource_name``: ros resource name to locate this icon

 * ``parameters``: is a free-form yaml variable used to configure the interaction in various ways:

   * *direct exes/rosrunnables/android app* : loads the yaml directly as private ros parameters.
   * *roslaunchables* : utilise key-value pairs only and passes these in as roslaunch args. 
   * *web app* : utilise key-value pairs only and reads these from the query string of the url.

 * ``remappings``: same meaning as remappings in `roslaunch xml`_ files, see the examples

   * *roslaunchables* : currently do not accept remappings (workaround: use parameters passed in as roslaunch args to configure topic names).

Interactions may be combined together in a yaml list for loading onto the interaction node.

Symbols
-------

The interactions node will substitute various symbols when loading the interactions. These
include:

 * ``ROSBRIDGE_ADDRESS`` : substitutes the interactions node's configured rosbridge address parameter
 * ``ROSBRIDGE_PORT`` : substitutes the interactions node's configured rosbridge port parameter

Examples
--------

Web app:

.. code-block:: yaml

   name: web_app(http://robotics-in-concert.github.io/rocon_tools/js/current/listener.html)
   role: 'Web Apps'
   compatibility: rocon:/*/*/hydro|indigo
   display_name: Listener
   description: Simple listener using rosjs in a web app.
   icon:
     resource_name: rocon_bubble_icons/rocon.png
   parameters: 
     rosbridge_address: __ROSBRIDGE_ADDRESS__
     rosbridge_port: __ROSBRIDGE_PORT__
   remappings:
     - remap_from: /chatter
       remap_to: /babbler

.. _`yaml`: http://en.wikipedia.org/wiki/YAML
.. _`rocon_uri`: http://wiki.ros.org/rocon_uri
.. _`roslaunch xml`: http://wiki.ros.org/roslaunch/XML/remap

