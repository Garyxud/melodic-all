#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
#
##############################################################################
##############################################################################
# Description
##############################################################################

"""
.. module:: utils
   :platform: Unix
   :synopsis: Utilities supporting the interactions classes.


This module defines utilities supporting interactions functionality.
----
"""
##############################################################################
# Imports
##############################################################################

import os
import genpy
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_python_utils
import rospkg
import yaml
import zlib  # crc32

from .exceptions import MalformedInteractionsYaml, YamlResourceNotFoundException

##############################################################################
# Request Interactions Resopnse
##############################################################################

_interaction_error_messages = {
    interaction_msgs.ErrorCodes.SUCCESS: 'Success',
    interaction_msgs.ErrorCodes.INTERACTION_UNAVAILABLE: interaction_msgs.ErrorCodes.MSG_INTERACTION_UNAVAILABLE,
    interaction_msgs.ErrorCodes.PAIRING_UNAVAILABLE: interaction_msgs.ErrorCodes.MSG_PAIRING_UNAVAILABLE,
    interaction_msgs.ErrorCodes.INTERACTION_QUOTA_REACHED: interaction_msgs.ErrorCodes.MSG_INTERACTION_QUOTA_REACHED,
    interaction_msgs.ErrorCodes.ALREADY_PAIRING: interaction_msgs.ErrorCodes.MSG_ALREADY_PAIRING,
    interaction_msgs.ErrorCodes.START_PAIRING_FAILED: interaction_msgs.ErrorCodes.MSG_START_PAIRING_FAILED,
    interaction_msgs.ErrorCodes.STOP_PAIRING_FAILED: interaction_msgs.ErrorCodes.MSG_STOP_PAIRING_FAILED,
    interaction_msgs.ErrorCodes.NOT_PAIRING: interaction_msgs.ErrorCodes.MSG_STOP_PAIRING_FAILED,
    interaction_msgs.ErrorCodes.REQUIRED_RAPP_IS_NOT_RUNNING: interaction_msgs.ErrorCodes.MSG_REQUIRED_RAPP_IS_NOT_RUNNING,
    interaction_msgs.ErrorCodes.DIFFERENT_RAPP_IS_RUNNING: interaction_msgs.ErrorCodes.MSG_DIFFERENT_RAPP_IS_RUNNING
}


def generate_request_interaction_response(code):
    """
    Construct according to the incoming code a default response message for the request interactions service.

    :param int code: one of the interaction_msgs.ErrorCodes types relevant to the set interactions service.
    :return: the response, filled with result code and message.
    :rtype: interaction_srvs.RequestInteractionResponse
    """
    response = interaction_srvs.RequestInteractionResponse()
    response.result = code
    response.message = _interaction_error_messages[code]
    return response

##############################################################################
# Hashing
##############################################################################


def generate_hash(display_name, group, namespace):
    '''
      Compute a unique hash for this interaction corresponding to the
      display_name-group-namespace triple. We use zlib's crc32 here instead of unique_id because
      of it's brevity which is important when trying to id an interaction by its hash
      from an nfc tag.

      Might be worth checking http://docs.python.org/2.7/library/zlib.html#zlib.crc32 if
      this doesn't produce the same hash on all platforms.

      :param str display_name: the display name of the interaction
      :param str group: the group the interaction is embedded in
      :param str namespace: the namespace in which to embed this interaction

      :returns: the hash
      :rtype: int32
    '''
    return zlib.crc32(display_name + "-" + group + "-" + namespace)


##############################################################################
# Loading From Files
##############################################################################

def load_msgs_from_yaml_file(file_path):
    """
      Load interactions from a yaml resource.

      :param str file_path: file path of a yaml formatted interactions file (ext=.interactions).

      :returns: a list of ros msg pairing-interaction specifications
      :rtype: (rocon_interaction_msgs.Pairing[], rocon_interaction_msgs.Interaction [])

      :raises: :exc:`.YamlResourceNotFoundException` if yaml is not found.
      :raises: :exc:`.MalformedInteractionsYaml` if yaml is malformed.

      .. include:: weblinks.rst
    """
    pairings = []
    interactions = []
    try:
        yaml_filename = file_path
        if not os.path.isfile(yaml_filename):
            raise YamlResourceNotFoundException(str(yaml_filename) + " NOT FOUND")
    except rospkg.ResourceNotFound as e:  # resource not found.
        raise YamlResourceNotFoundException(str(e))
    with open(yaml_filename) as f:
        # load the interactions from yaml into a python object
        yaml_objects = yaml.load(f)
        try:
            pairing_yaml_objects = yaml_objects['pairings']
            for pairing_yaml_object in pairing_yaml_objects:
                pairing = interaction_msgs.Pairing()
                try:
                    genpy.message.fill_message_args(pairing, pairing_yaml_object)
                except genpy.MessageException as e:
                    raise MalformedInteractionsYaml(
                        "malformed yaml preventing converting of yaml to pairing msg [%s]" % str(e))
                pairings.append(pairing)
        except KeyError:
            # probably just interactions in this yaml file - not an error, so just continue
            pass
        try:
            interaction_yaml_objects = yaml_objects['interactions']
            if interaction_yaml_objects is not None:
                # now drop it into message format
                for interaction_yaml_object in interaction_yaml_objects:
                    # convert the parameters from a freeform yaml variable to a yaml string suitable for
                    # shipping off in ros msgs (where parameters is a string variable)
                    if 'parameters' in interaction_yaml_object:  # it's an optional key
                        # chomp trailing newlines
                        interaction_yaml_object['parameters'] = yaml.dump(interaction_yaml_object['parameters']).rstrip()
                    interaction = interaction_msgs.Interaction()
                    try:
                        genpy.message.fill_message_args(interaction, interaction_yaml_object)
                    except genpy.MessageException as e:
                        raise MalformedInteractionsYaml(
                            "malformed yaml preventing converting of yaml to interaction msg [%s]" % str(e))
                    interactions.append(interaction)
        except KeyError:
            # probably just pairings in this yaml file - not an error, so just continue
            pass
    return (pairings, interactions)


def load_msgs_from_yaml_resource(resource_name):
    """
      Load interactions from a yaml resource.

      :param str resource_name: pkg/filename of a yaml formatted interactions file (ext=.interactions).

      :returns: a list of ros msg pairing-interaction specifications
      :rtype: (rocon_interaction_msgs.Pairing[], rocon_interaction_msgs.Interaction [])

      :raises: :exc:`.YamlResourceNotFoundException` if yaml is not found.
      :raises: :exc:`.MalformedInteractionsYaml` if yaml is malformed.

      .. include:: weblinks.rst
    """
    interactions = []
    try:
        yaml_filename = rocon_python_utils.ros.find_resource_from_string(resource_name, extension='interactions')
        (pairings, interactions) = load_msgs_from_yaml_file(yaml_filename)
        return (pairings, interactions)
    except rospkg.ResourceNotFound as e:  # resource not found.
        raise YamlResourceNotFoundException(str(e))
