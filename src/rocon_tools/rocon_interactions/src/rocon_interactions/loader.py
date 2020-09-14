#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: loader
   :platform: Unix
   :synopsis: External loader of interactions.


This module provides a class that lets you conveniently load interactions
from outside the interactions manager node post startup (i.e. not using params).
This class is the skeleton of the ``load_interactions`` script which can be used
in a roslaunch file in the following way:

.. code-block:: xml

   <!-- instead of params, use the external loader and configure groups with default namespaces -->
   <node pkg="rocon_interactions" type="load_interactions" name="load_interactions" args="-n '/web' rocon_interactions web">
     <remap from="load_interactions/set_interactions" to="interactions/set_interactions"/>
   </node>

----

"""
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_python_comms

from . import interactions
from . import utils

##############################################################################
# Loader
##############################################################################


class InteractionsLoader(object):

    '''
      This class is responsible for loading the role manager with the roles
      and app specifications provided in the service definitions.
    '''
    __slots__ = [
        '_set_interactions_proxy',
    ]

    def __init__(self):
        '''
        Don't do any loading here, just set up infrastructure and overrides from
        the solution.

        :raises: rocon_python_comms.NotFoundException, rospy.exceptions.ROSException,
               rospy.exceptions.ROSInterruptException
        '''
        try:
            service_name = rocon_python_comms.find_service('rocon_interaction_msgs/SetInteractions',
                                                           timeout=rospy.rostime.Duration(15.0),
                                                           unique=True)
        except rocon_python_comms.NotFoundException as e:
            raise rocon_python_comms.NotFoundException(
                "failed to find unique service of type 'rocon_interaction_msgs/SetInteractions' [%s]"
                % str(e))
        self._set_interactions_proxy = rospy.ServiceProxy(service_name, interaction_srvs.SetInteractions)

    def load_from_file(self, interactions_yaml_filepath, namespace='/', load=True):
        '''
        Parse a set of configurations specified in a yaml file found from file path
        and send the command toload/unload these on the interactions manager.
        For convenience, it also allows the setting of a namespace for the whole group
        which will only get applied if an interaction has no setting in the yaml.

        :param str interactions_yaml_filepath: yaml absolute file path for role-app parameterisation
        :param str namespace: namespace to push connections down into (e.g. /interactions)
        :param bool load: either load or unload the interaction information.

        :raises: :exc:`.YamlResourceNotFoundException`, :exc:`.MalformedInteractionsYaml`
        '''
        request = interaction_srvs.SetInteractionsRequest()
        request.load = load

        # This can raise YamlResourceNotFoundException, MalformedInteractionsYaml
        request.interactions = interactions.load_msgs_from_yaml_file(interactions_yaml_filepath)

        for i in request.interactions:
            if i.namespace == '':
                i.namespace = namespace

        # Should check the response here and return some sort of true/false result.
        unused_response = self._set_interactions_proxy(request)

    def load_from_resource(self, interactions_yaml_resource, namespace='/', load=True):
        '''
        Parse a set of configurations specified in a yaml file found from resource (package/file name pair)
        and send the command to load/unload these on the interactions manager.
        For convenience, it also allows the setting of a namespace for the whole group
        which will only get applied if an interaction has no setting in the yaml.

        :param str interactions_yaml_resource: yaml resource name for role-app parameterisation
        :param str namespace: namespace to push connections down into (e.g. /interactions)
        :param bool load: either load or unload the interaction information.

        :raises: :exc:`.YamlResourceNotFoundException`, :exc:`.MalformedInteractionsYaml`
        '''
        request = interaction_srvs.SetInteractionsRequest()
        request.load = load

        # This can raise YamlResourceNotFoundException, MalformedInteractionsYaml
        (request.pairings, request.interactions) = utils.load_msgs_from_yaml_resource(interactions_yaml_resource)

        for i in request.interactions:
            if i.namespace == '':
                i.namespace = namespace

        # Should check the response here and return some sort of true/false result.
        unused_response = self._set_interactions_proxy(request)
