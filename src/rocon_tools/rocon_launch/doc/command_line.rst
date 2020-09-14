Command Line Usage
==================

The ``rocon_launch`` tool executes ``.concert`` files which are effectively an xml
formatted super-ros-launcher. 

Invocation
----------

Executing ``rocon_launch`` is very similar to roslaunch_
but has some notable differences. Typically it will be run via a reference to a package::

    > rocon_launch package_name file.concert

or directly::

    > rocon_launch file.concert

Options
-------

**Force a specific terminal**

::

     -k, --konsole   spawn individual ros systems via multiple konsole terminals
     -g, --gnome     spawn individual ros systems via multiple gnome terminals

**Verbose Roslaunch Output**

::

     --screen        run each roslaunch with the --screen option

**Single Terminal**

::

     --no-terminals  do not spawn terminals for each roslaunch

**Keep Terminals Open**

This is useful for debugging roslaunched environments that are having problems shutting down.

::

     --hold          hold terminals open after upon completion (incompatible with
                     --no-terminals)

Arguments
---------

Arguments can be specified on the command line in the same way as roslaunch:: 

    > rocon_launch package_name file.concert my_arg:=my_value

See also the tag :ref:`arg-section-label` definition.


.. _roslaunch: http://wiki.ros.org/roslaunch

