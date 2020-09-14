RosApi
======

These are the specifications for the ros api of the interaction node. This is
of interest for people using the api to (un)load interactions and also for the
developer creating remocons (interaction clients).

Published Topics
----------------

 * ``~interactive_clients`` (`rocon_interaction_msgs`_/InteractiveClients) : introspect the list of connected remocons, latched.
 * ``~pairing`` (`rocon_interaction_msgs`_/Pair) : if a paired interaction is running, it publishes the names of the pair here (used by rapp manager and remocon for monitoring), latched.

Subscribed Topics
-----------------

 * ``~status`` (`rocon_app_manager_msgs`_/Status) : for pairing interactions, monitor the status of a running rapp so it can terminate the interaction if necessary.

Services
--------

 * ``~get_interaction`` (`rocon_interaction_msgs`_/GetInteraction) : used by the android headless launcher to get a single interactions details by hash.
 * ``~get_interactions`` (`rocon_interaction_msgs`_/GetInteractions) : used by all remocons to get a filtered set of compatible interactions by role and `rocon_uri`_.
 * ``~set_interactions`` (`rocon_interaction_msgs`_/SetInteractions) : used to load and unload interactions on the interaction manager node.
 * ``~get_roles`` (`rocon_interaction_msgs`_/GetRoles) : introspect the currently loaded roles filtered by a rocon uri ('' or 'rocon:/' to get all).
 * ``~request_interaction`` (`rocon_interaction_msgs`_/RequestInteraction) : used by remocons to request permission to start an interaction (can be denied - too many instances, no rapp manager...).

.. _`rocon_app_manager_msgs`: http://wiki.ros.org/rocon_app_manager_msgs
.. _`rocon_interaction_msgs`: http://wiki.ros.org/rocon_interaction_msgs
.. _`rocon_master_info`: http://wiki.ros.org/rocon_master_info
.. _`rocon_std_msgs`: http://wiki.ros.org/rocon_std_msgs
.. _`rocon_uri`: http://wiki.ros.org/rocon_uri

Parameters
----------

 * ``~rosbridge_address`` : if serving web apps, pop the rosbridge address here (default: localhost).
 * ``~rosbridge_address`` : if serving web apps, pop the rosbridge port here (default: 9090).
  