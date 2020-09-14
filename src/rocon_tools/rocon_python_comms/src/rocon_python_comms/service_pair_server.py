#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: service_pair_server
   :platform: Unix
   :synopsis: Server side api for communicating across a rocon service pair.


This module contains the server side api for communicating across a rocon
service pair. A `facade pattern`_ is used here to simplify the interaction with
the server side publisher and subscriber.

.. include:: weblinks.rst

----

"""
##############################################################################
# Imports
##############################################################################

import rospy
import threading

# Local imports
from .exceptions import ServicePairException

##############################################################################
# Server Class
##############################################################################


class ServicePairServer(object):
    '''
      The server side of a pubsub service pair. This class provides a simplified
      api for handling requests/responses on the pubsub pair. There are two
      modes of operation - 1) blocking and 2) threaded.

      **Non-Threaded**

      In the first, the users' callback function directly runs whenever an
      incoming request is received. In this case, your callbacks should be
      very minimal so that incoming requests don't get blocked and queued up.

      .. code-block:: python

            #!/usr/bin/env python

            import rospy
            from chatter.msg import ChatterRequest, ChatterResponse, ChatterPair
            from rocon_python_comms import ServicePairServer

            class ChatterServer(object):

                def __init__(self):
                    self.server = ServicePairServer('chatter', self.callback, ChatterPair)

                def callback(self, request_id, msg):
                    rospy.loginfo("Server : I heard %s" % msg.babble)
                    response = ChatterResponse()
                    response.reply = "I heard %s" % msg.babble
                    self.server.reply(request_id, response)

            if __name__ == '__main__':
                rospy.init_node('chatter_server', anonymous=True)
                chatter_server = ChatterServer()
                rospy.spin()

      **Threaded**

      In the second, we spawn a background thread and shunt the callback into this thread.
      Just toggle the ``use_threads`` flag when constructing the server:

      .. code-block:: python

          self.server = ServicePairServer('chatter', self.callback, ChatterPair, use_threads=True)
    '''
    __slots__ = [
            '_publisher',
            '_subscriber',
            '_callback',
            '_use_threads',
            #'_request_handlers',  # initiate, track and execute requests with these { hex string ids : dic of RequestHandler objects (Blocking/NonBlocking) }
            'ServicePairSpec',
            'ServicePairRequest',
            'ServicePairResponse',
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, name, callback, ServicePairSpec, use_threads=False, queue_size=5):
        '''
        :param str name: resource name of service pair (e.g. testies for pair topics testies/request, testies/response)
        :param callback: function invoked when a request arrives
        :param ServicePairSpec: the pair type (e.g. rocon_service_pair_msgs.msg.TestiesPair)
        :param bool use_threads: put the callback function into a fresh background thread when a request arrives.
        :param int queue_size: size of the queue to configure the publisher with.
        '''
        self._callback = callback
        self._use_threads = use_threads
        try:
            p = ServicePairSpec()
            self.ServicePairSpec = ServicePairSpec
            """Base message type for this pair."""
            self.ServicePairRequest = type(p.pair_request)
            """Request msg type for this pair <ServicePairSpec>Request."""
            self.ServicePairResponse = type(p.pair_response)
            """Response msg type for this pair <ServicePairSpec>Response."""
        except AttributeError:
            raise ServicePairException("Type is not an pair spec: %s" % str(ServicePairSpec))
        self._subscriber = rospy.Subscriber(name + "/request", self.ServicePairRequest, self._internal_callback)
        self._publisher = rospy.Publisher(name + "/response", self.ServicePairResponse, queue_size=queue_size)

    ##########################################################################
    # Public Methods
    ##########################################################################

    def reply(self, request_id, msg):
        '''
        Send a reply to a previously received request (identified by request_id). Use this
        instead of writing directly to the publisher - just pass the content of the
        response data and the id that was issued with the request.

        :param uuid_msgs.UniqueID request_id: the request id to associate with this response.
        :param ServiceResponse msg: the response
        '''
        pair_response = self.ServicePairResponse()
        pair_response.id = request_id
        pair_response.response = msg
        self._publisher.publish(pair_response)

    ##########################################################################
    # Callbacks
    ##########################################################################

    def _internal_callback(self, msg):
        '''
        :param ServicePairRequest msg: message returned from the server (with pair id etc)
        '''
        # Check if it is a blocking call that has requested it.
        if self._use_threads:
            thread = threading.Thread(target=self._callback, args=(msg.id, msg.request))
            thread.start()
        else:
            self._callback(msg.id, msg.request)
