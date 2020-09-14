=======
rtctree
=======

Introduction
============

rtctree is a Python library providing an easy-to-use API for interacting
with running RT Components and RTM-based systems running on
OpenRTM-aist-1.0. It allows developers to manage these systems from
other programs without needing to learn the CORBA API. Components can be
started, stopped, connected together, have their configuration changed,
and so on.

This software is developed at the National Institute of Advanced
Industrial Science and Technology. Approval number H23PRO-1229. The
development was financially supported by the New Energy and Industrial
Technology Development Organisation Project for Strategic Development of
Advanced Robotics Elemental Technologies.  This software is licensed
under the Eclipse Public License -v 1.0 (EPL). See LICENSE.TXT.


Requirements
============

rtctree requires omniorb-py, including omniidl with the Python backend.
If you have installed OpenRTM-python, you will have these installed
already. If not, you will need to install them manually.

rtctree uses the new string formatting operations that were introduced
in Python 2.6. It will not function with an earlier version of Python.
It has not been tested with Python 3 and it is likely that several
changes will be necessary to make it function using this version of
Python.

For Ubuntu users, if you are using a version of Ubuntu prior to 9.04,
you will need to install a suitable Python version by hand. You may want
to consider upgrading to Ubuntu 9.04 or later (10.04 offers LTS).


Installation
============

There are several methods of installation available:

 1. Download the source from either the repository (see "Repository,"
 below) or a source archive, extract it somewhere, and install it into
 your Python distribution:

   a) Extract the source, e.g. to a directory /home/blag/src/rtctree

   b) Run setup.py to install rtctree to your default Python
   installation::

      $ python setup.py install

   c) If necessary, set environment variables. These should be set by
   default, but if not you will need to set them yourself. On Windows,
   you will need to ensure that your Python site-packages directory is
   in the PYTHONPATH variable and the Python scripts directory is in the
   PATH variable.  Typically, these will be something like
   ``C:\Python26\Lib\site-packages\`` and ``C:\Python26\Scripts\``,
   respectively (assuming Python 2.6 installed in ``C:\Python26\``).

 2. Use the Windows installer. This will perform the same job as running
 setup.py (see #2), but saves opening a command prompt. You may still
 need to add paths to your environment variables (see step c, above).


Environment variables
=====================

The following environment variables are used:

  ``RTCTREE_ORB_ARGS``
    A list of arguments, separated by semi-colons, to pass to the ORB
    when creating it. Optional.

  ``RTCTREE_NAMESERVERS``
    A list of name server addresses, separated by semi-colons, to parse
    when creating the RTCTree. Each server in the list will be added to
    the tree. Optional.

The only variable that should normally be set by the user is
``RTCTREE_NAMESERVERS``. Set this to a list of name server addresses,
separated by semi-colons, that you want rtcshell to interact with. For
example, in a Bash shell, you can run the following::

 $ export RTCTREE_NAMESERVERS=localhost;192.168.0.1:65346;example.com


The RTC Tree
============

The core of the library is the RTC Tree::

  import rtctree.tree
  tree = rtctree.tree.RTCTree()

This is a file system-like tree built by parsing name servers to find
directories, components and managers. You can treat it exactly the same
way as you treat a normal file system. The tree represents the naming
contexts, managers and components registered all on known name servers
in a tree structure::

 \
 |-+localhost
 | |-+naming_context
 | | |--ConsoleIn0.rtc
 | | |--ConsoleOut0.rtc
 | |
 | |--another_naming_context
 | |--Sensor0.rtc
 |
 |-+192.168.0.5
   |--Motor0.rtc
   |--Controller0.rtc

Each ``directory`` in the tree represents a naming context, which may be
a normal naming context or the root context of a name server. These are
represented by NameServer and Directory objects.

Name servers are treated as directories off the root directory, ``/``.
Below them are ``files`` and sub-directories. A sub-directory represents
a naming context below the root naming context of a name server.

Files are components and managers, represented by the Component and
Manager classes, respectively.

Component objects store a variety of information about the component
they represent. You can access the component's ports, configuration
sets, and so on.  Use these objects to change configuration values,
connect ports to each other, start and stop components, etc.

All nodes in the tree also store the CORBA object reference to the
object they represent. By accessing this object, you can call the IDL
methods. If something is not currently available in rtctree, calling the
IDL method on the CORBA object directly will be able to achieve what you
want to do.


Building the tree
-----------------

The arguments to the tree factory function determine which name servers
are parsed to build the tree. See that function's documentation for
details. In general, you can pass in a list of server addresses and/or a
list of paths (the first component of each path is treated as a name
server). The environment variable ``RTCTREE_NAMESERVERS`` will also be
checked for any additional name servers to parse. This is a semi-colon
separated list of name server addresses.


Paths
-----

Nodes in the tree are addressed using paths. A path is a list of
strings, each representing a level in the tree one deeper than the
previous list item.  Absolute paths are necessary to address into the
tree object. Addressing from nodes allows relative paths, provided that
the path exists below the node.

When represented as text, these paths resemble file system paths. The
root of the tree is represented by ``/`` (``\`` on Windows systems). The
first level of entries are name server addresses. Entries below the
first level are components, managers and naming contexts (which are
represented as directories). The utility function parse_path will parse
a text string path into a list of path entries that can be used to
address nodes in the tree.

For example, the path ``/localhost/naming_context/ConsoleIn0.rtc``
represents the component ``ConsoleIn0.rtc``, registered in the
``naming_context naming`` context on the name server running at
``localhost``. When used to find the node in the tree representing this
component, the path should be a Python list::

  ['/', 'localhost', 'naming_context', 'ConsoleIn0.rtc']


Useful functions
----------------

Useful member functions of the RTCTree class and node classes that will
be of particular interest are shown below. This is not a complete list
of all available functionality. Users are encouraged to check the full
API documentation for additional functionality, and examine the rtcshell
source code for usage examples.

  ``RTCTree.has_path``
    Checks if a path is present in the tree.  Use this to quickly check
    if a component exists.
  ``RTCTree.get_node``
    Retrieves a node from the tree based on a path. Use this to get
    components, directories, etc. from the tree.
  ``RTCTree.is_component``
    Tests if the given path points to a Component object.  Tree nodes
    have a property, is_component, that performs the same function
    directly on a node. is_directory, is_manager and is_nameserver
    functions and properties are also available.
  ``RTCTree.iterate()``
    Use this function to perform an action on every node in the tree, or
    only those nodes matching a given filter. The return result of each
    call will be returned from iterate as a list.  This function is
    particularly useful. See rtcshell's rtls command for an example of
    using iterate().


  ``Node.children``
    This property gives a list of the node's children. You can use this,
    for example, to get all the components in a directory of the tree.
  ``Node.full_path``
    The full path of the node from the root of the tree.
  ``Node.name``
    The name of this node; i.e. its entry in the tree.
  ``Node.parent_name``
    The name of this node's parent (if it has one).
  ``Node.root``
    Given a node, use this property to get the root node of the tree it
    is in, on which you can perform nearly all functions you can perform
    on the tree object.


  ``Component.activate_in_ec()``
    Activate the component in the execution context at the given index.
    For most components, only one EC is present and so the index should
    be 0.
  ``Component.deactivate_in_ec()``
    Deactivate the component in an execution context.
  ``Component.reset_in_ec()``
    Reset the component in an execution context.
  ``Component.state_in_ec()``
    Get the state in a specific execution context.
  ``Component.alive``
    Test if the component is alive.
  ``Component.owned_ecs``
    The list of execution contexts owned by the component.
  ``Component.participating_ecs``
    The list of execution contexts the component is participating in.
  ``Component.state``
    The overall state of the component, created by merging its state in
    each execution context.
  ``Component.state_string``
    The overall state of the component as a string.
  ``Component.disconnect_all()``
    Disconnect all connections from all ports of this component.
  ``Component.get_port_by_name()``
    Find a port of this component by name.
  ``Component.ports``
    The list of the component's ports. Similar lists exist for input,
    output and service ports.  Component.connected_ports The list of the
    component's ports that are connected. Similar lists exist for
    connected input, output and service ports.
  ``Component.object``
    Get the CORBA LightweightRTObject that this component wraps.
  ``Component.activate_conf_set``
    Activate a configuration set by name.
  ``Component.set_conf_set_value``
    Set the value of a parameter in a configuration set.
  ``Component.active_conf_set``
    The currently-active configuration set.
  ``Component.active_conf_set_name``
    The name of the currently-active configuration set.
  ``Component.conf_sets``
    The list of configuration sets.


  ``Port.connect()``
    Connect this port to another port.
  ``Port.disconnect_all()``
    Disconnect all connections on this port.
  ``Port.get_connection_by_dest()``
    Get a connection on this port by the destination port.
  ``Port.get_connection_by_name()``
    Get a connection on this port by its name.
  ``Port.connections``
    The connections on this port.
  ``Port.is_connected``
    Checks if this port is connected or not.
  ``Port.name``
    The name of this port.
  ``Port.object``
    The CORBA PortService object that this component wraps.
  ``Port.name``
    The port's owner (usually a Component object).
  ``Port.porttype``
    The type of the port (DataInPort, DataOutPort or CorbaPort).


  ``Connection.disconnect()``
    Remove this connection between ports.
  ``Connection.ports``
    The list of ports involved in this connection.

  ``ConfigurationSet.has_param()``
    Checks if a parameter is present in the configuration set.
  ``ConfigurationSet.set_param()``
    Sets the value of a parameter in this configuration set.


  ``ExecutionContext.activate_component()``
    Activate a component within this execution context.
  ``ExecutionContext.deactivate_component()``
    Deactivate a component within this execution context.
  ``ExecutionContext.reset_component()``
    Reset a component within this execution context.
  ``ExecutionContext.get_component_state()``
    Get the state of a component within this execution context.
  ``ExecutionContext.running``
    Check if this execution context is running or not.


  ``Manager.create_component()``
    Create a new component instance.
  ``Manager.delete_component()``
    Destroy a component instance.


  ``dict_to_nvlist()``
    Converts a Python dictionary into a CORBA namevalue list.
  ``nvlist_to_dict()``
    Converts a CORBA namevalue list into a Python dictionary.


API naming conventions
======================

rtctree follows the standard Python naming conventions as laid out in
PEP8_.

Most importantly, the private, internal API functions begin with an
underscore (``_``). If a function begins with an underscore, it is not
intended for use outside the class and doing so could lead to undefined
behaviour. Only use those API functions that do not begin with an
underscore and have a docstring in your programs.

.. _PEP8: http://www.python.org/dev/peps/pep-0008/


Further documentation and examples
==================================

For further documentation, see the generated API documentation.

For examples, see the rtshell set of utilities. These illustrate using
rtctree to perform most of the actions possible using RTSystemEditor.


Repository
==========

The latest source is stored in a Git repository at github_.  You can
download it as a zip file or tarball by clicking the "Download Source"
link in the top right of the page.  Alternatively, use Git to clone the
repository. This is better if you wish to contribute patches.

::

 $ git clone git://github.com/gbiggs/rtctree.git

.. _github: http://github.com/gbiggs/rtctree


Changelog
=========

4.0
---

 - Added complete support for the ExecutionContext interface.
 - Support for the Logger SDO interface.
 - Support for the ComponentObserver SDO interface.
 - Support three-or-more port connections.
 - Deprecated Port.get_connection_by_dest()
 - Added Port.get_connections_by_dest()
 - Added Port.get_connections_by_dests()

3.0.1
-----

- Compatibility release for rtshell-3.0.1.

3.0
---

 - Do not treat exceptions while parsing managers as fatal
 - Other zombie-catching improvements
 - Detect zombie managers
 - New API calls to get composite component information
 - New API call to get a connection from a port by ID
 - Added API ability to give away the ORB
 - Added path formatter
 - Pretty-print exceptions
 - Other performance improvements (e.g. removed double parsing)
 - Added ability to restrict parsed paths (improves startup speed)
 - Added Zombie node
 - New API call to make a component exit
 - Removed defunct create_rtctree call
 - Exposed remove_node API call
 - Changed node.full_path to return a list, added node.full_path_str


2.0
---

 - Parse more information about execution contexts
 - Added the ability to use a provided ORB instead of creating one
 - Exposed the reparse_connections() method
 - New API call to get the ORB used for a node
 - New API call to unbind a name from a context
 - Allow access to more CORBA objects
 - Catch more zombies
 - Handle unknown CORBA object types
 - Handle the case of unknown port owners
 - Added locks to make rtctree objects thread-safe
 - Added API for forcing a re-parse of any object in the tree
 - Cleaned up ``__init__`` functions for proper inheritence handling
 - New API call to get the state of a component in a specific EC
 - New API call to update the state of a component in a specific EC

