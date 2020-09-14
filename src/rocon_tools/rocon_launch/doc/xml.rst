==========
Xml Format
==========

Naming
======

By convention ``rocon_launch`` files are named with the extension ``.concert``. These
are able to be tab-completed on the command line when executing ``rocon_launch``. 

Tags
====

concert
-------

**Attributes**

None

**Elements**

* <:ref:`launch-section-label`> - describe a roslaunch environment
* <:ref:`arg-section-label`> - declare a roslaunch style argument_.

.. _launch-section-label:

launch
------


**Attributes**

* *title* - string to use to set the title of the terminal in the window decoration, this helps you quickly identify windows when tabbing through them.
* *package* - name of the package in which the roslaunch file can be found (optional)
* *name* - name of the roslaunch file to execute (if no package, then this must be a full path to the roslauncher)
* *port* - identifies which ros master to roslaunch in (i.e. ROS_MASTER_URI=http://localhost:_port_) 

**Elements**

* <:ref:`arg-section-label`> - pass an argument into the roslauncher.

.. _arg-section-label:

arg
---

**Usage**

This tag has exactly the same properties as the roslaunch `arg <http://wiki.ros.org/roslaunch/XML/arg>`_ tag. They can be used in one of three ways

::

    <arg name="foo" />

Declares the existence of foo. foo must be passed in either as a command-line argument (if top-level) or via <include> passing (if included).

::

   <arg name="foo" default="1" />

Declares foo with a default value. foo can be overriden by command-line argument (if top-level) or via <include> passing (if included).

::

    <arg name="foo" value="bar" />

Declares foo with constant value. The value for foo cannot be overridden. This usage enables internal parameterization of a launch file without exposing that parameterization at higher levels.

**Attributes**

* *name* - name of th argument
* *default* - default value (optional)
* *value* - argument value, cannot be defined with the default attribute (optional)

**Elements**

None

.. _argument : http://wiki.ros.org/roslaunch/XML/arg

Examples
========

Single master multi-launcher running talker in one window and listener in another. The topic name can be pre-configured via the ``topic_name`` arg.

.. code-block:: xml
   :linenos:

   <concert>
     <arg name="topic_name" default="chatter"/>
   
     <launch title="talker" package="rocon_launch" name="talker.xml" port="11311">
       <arg name="topic_name" value="$(arg topic_name)"/>
     </launch>
     <launch title="listener" package="rocon_launch" name="listener.xml" port="11311">
       <arg name="topic_name" value="$(arg topic_name)"/>
     </launch>
   </concert>

A multi-master chatter concert:

.. code-block:: xml
   :linenos:

   <concert>
     <arg name="local_machine_only" default="true"/>  <!-- only invite clients if they are on the same pc -->
   
     <launch title="concert:11311" package="chatter_concert" name="concert.launch" port="11311">
       <arg name="local_machine_only" value="$(arg local_machine_only)"/>
     </launch>
     <launch title="dudette:11312"    package="chatter_concert" name="dudette.launch" port="11312"/>
     <launch title="dude:11313"    package="chatter_concert" name="dude.launch" port="11313"/>
     <launch title="dude:11314"    package="chatter_concert" name="dude.launch" port="11314"/>
   </concert>
