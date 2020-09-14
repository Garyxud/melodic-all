#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: service_pair_client
   :platform: Unix
   :synopsis: Client side api for communicating across a rocon service pair.


This module contains the client side api for communicating across a rocon
service pair. A `facade pattern`_ is used here to simplify the interaction with
the client side publisher and subscriber.

.. include:: weblinks.rst

----

"""
##############################################################################
# Imports
##############################################################################

import threading
import time
import rospy
import uuid
import unique_id
import functools

# Local imports
from .exceptions import ServicePairException

##############################################################################
# Request Handlers
##############################################################################


class RequestHandlerBase(object):

    def __init__(self, key):
        self.key = key  # uuid hexstring key (the request_handlers key)
        self.response = None


class BlockingRequestHandler(RequestHandlerBase):

    def __init__(self, key):
        super(BlockingRequestHandler, self).__init__(key)
        self.event = threading.Event()


class NonBlockingRequestHandler(RequestHandlerBase):

    def __init__(self, key, callback, error_callback):
        super(NonBlockingRequestHandler, self).__init__(key)
        self.timer = None
        self.callback = callback
        self.error_callback = error_callback

    def copy(self):
        '''
          The deepcopy function has some issues (related to threads),
          so using this independant copy method here. Note that this only
          ever gets used for non-blocking calls to help handle the
          race conditions between timeout handling and normal callback
          handling
        '''
        new_copy = NonBlockingRequestHandler(self.key, self.callback, self.error_callback)
        new_copy.timer = self.timer
        return new_copy

##############################################################################
# Client Class
##############################################################################


class ServicePairClient(object):
    '''
      The client side of a pubsub service pair.This class provides a simplified
      api for handling requests/responses on the pubsub pair (client side).
      Although the development of this class was for non-blocking request-response
      behaviour, it does support legacy style blocking behavious as well.

      **Usage (Non-Blocking):**

      .. code-block:: python

          #!/usr/bin/env python

          import rospy
          from chatter.msg import ChatterRequest, ChatterResponse, ChatterPair
          from rocon_python_comms import ServicePairClient
          import unique_id

          class ChatterClient(object):

              def __init__(self):
                  self.response = None
                  self.client = ServicePairClient('chatter', ChatterPair)
                  self.client.wait_for_service(rospy.Duration(3.0)) # should catch some exceptions here in case of timeouts
                  self.request_id = self.client(ChatterRequest("Hello dude"), timeout=rospy.Duration(3.0), callback=self.callback)
                  rospy.loginfo("Request id %s" % unique_id.toHexString(self.request_id))
                  if self.response is not None:
                      print("Response %s" % self.response)

              def callback(self, request_id, msg):
                  # ideally you'd check request_id against self.request_id
                  self.response = msg
                  rospy.loginfo("Got the response: %s" % msg)

          if __name__ == '__main__':
              rospy.init_node('chatter_client', anonymous=True)
              chatter_client = ChatterClient()
              rospy.spin()

      **Usage (Blocking):**

      .. code-block:: python

         #!/usr/bin/env python

         import rospy
         from chatter.msg import ChatterRequest, ChatterResponse, ChatterPair
         from rocon_python_comms import ServicePairClient
         import unique_id

         class ChatterClient(object):

             def __init__(self):
                 self.response = None
                 self.client = ServicePairClient('chatter', ChatterPair)
                 self.client.wait_for_service(rospy.Duration(3.0)) # should catch some exceptions here in case of timeouts
                 self.response = self.client(ChatterRequest("Hello dude"), timeout=rospy.Duration(3.0))
                 if self.response is not None:
                     print("Response %s" % self.response)

         if __name__ == '__main__':
             rospy.init_node('chatter_client', anonymous=True)
             chatter_client = ChatterClient()
             rospy.spin()
    '''
    __slots__ = [
            '_publisher',
            '_subscriber',
            '_request_handlers',  # initiate, track and execute requests with these { hex string ids : dic of RequestHandler objects (Blocking/NonBlocking) }
            'ServicePairSpec',
            'ServicePairRequest',
            'ServicePairResponse',
            '_lock'               # prevent race conditions in handling of non-blocking callbacks and timeouts.
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, name, ServicePairSpec):
        '''
          :param str name: resource name of service pair (e.g. testies for pair topics testies/request, testies/response)
          :param str ServicePairSpec: the pair type (e.g. rocon_service_pair_msgs.msg.TestiesPair)
        '''
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
        self._lock = threading.Lock()
        self._subscriber = rospy.Subscriber(name + "/response", self.ServicePairResponse, self._internal_callback)
        self._publisher = rospy.Publisher(name + "/request", self.ServicePairRequest, queue_size=5)
        self._request_handlers = {}  # [uuid_msgs/UniqueId]

    def wait_for_service(self, timeout=None):
        '''
        Waits for the service pair server to appear. The input argument to timeout
        modifies the behaviour of this function as demonstrated below.

        .. code-block:: python

           # one-shot
           if client.wait_for_service():
               # do something useful

           # timeout
           if not client.wait_for_service(rospy.Duration(5.0)):
               rospy.logwarn("Timed out waiting for service to appear")

           # indefinite blocking
           client.wait_for_service(rospy.Duration(0))

        :param rospy.Duration timeout: time to wait for data
        :returns: bool true if connections found, false otherwise
        :raises: rospy.ROSInterruptException if shutdown interrupts wait
        '''
        if timeout is None:
            return self._subscriber.get_num_connections() > 0 and self._publisher.get_num_connections() > 0

        timeout_time = time.time() + timeout.to_sec()
        while not rospy.is_shutdown():
            if timeout > rospy.Duration(0) and time.time() > timeout_time:
                return False
            if self._subscriber.get_num_connections() > 0 and self._publisher.get_num_connections() > 0:
                return True
            rospy.rostime.wallsleep(0.1)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException("rospy shutdown")
        return False

    ##########################################################################
    # Execute Blocking/NonBlocking
    ##########################################################################

    def __call__(self, msg, timeout=None, callback=None, error_callback=None):
        '''
          Initiates and executes the client request to the server. The type of arguments
          supplied determines whether to apply blocking or non-blocking behaviour.

          If no callback is supplied, the mode is blocking, otherwise non-blocking.
          If no timeout is specified, then a return of None indicates that the
          operation timed out.

          :param msg: the request message
          :type msg: <name>Request

          :param rospy.Duration timeout: time to wait for data

          :param callback: user callback invoked for responses of non-blocking calls
          :type callback: method with arguments (uuid_msgs.UniqueID, <name>Response)

          :returns: msg/id for blocking calls it is the response message, for non-blocking it is the unique id
          :rtype: <name>Response/uuid_msgs.UniqueID or None (if timed out)
        '''
        pair_request_msg = self.ServicePairRequest()
        pair_request_msg.id = unique_id.toMsg(unique_id.fromRandom())
        pair_request_msg.request = msg
        key = unique_id.toHexString(pair_request_msg.id)
        if callback == None and error_callback == None:
            self._request_handlers[key] = BlockingRequestHandler(key)
            return self._make_blocking_call(self._request_handlers[key], pair_request_msg, timeout)
        else:
            request_handler = NonBlockingRequestHandler(key, callback, error_callback)
            self._request_handlers[key] = request_handler.copy()
            self._make_non_blocking_call(request_handler, pair_request_msg, timeout)
            return pair_request_msg.id

    ##########################################################################
    # Private Support Methods
    ##########################################################################

    def _make_blocking_call(self, request_handler, msg, timeout):
        '''
          :param request_handler: information and event handler for the request
          :type request_handler: :class:`.RequestHandler`

          :param msg: the request pair message structure
          :type msg: ServicePairRequest
        '''
        self._publisher.publish(msg)
        if timeout is None:
            request_handler.event.wait()
        else:
            request_handler.event.wait(timeout.to_sec())
        if request_handler.response is not None:
            response = request_handler.response
        else:
            response = None
        del self._request_handlers[request_handler.key]
        return response

    def _make_non_blocking_call(self, request_handler, msg, timeout):
        '''
          :param request_handler: a copy of information and event handler for the request (used for the timer)
          :type request_handler: :class:`.RequestHandler`

          :param msg: the request pair message structure
          :type msg: ServicePairRequest
        '''
        self._publisher.publish(msg)
        if timeout is not None:
            # bind the key so the timer callback knows which request to handle.
            delete_request_handler = functools.partial(self._timer_callback, request_handler=request_handler)
            request_handler.timer = rospy.Timer(timeout, delete_request_handler, oneshot=True)

    def _timer_callback(self, unused_event, request_handler):
        '''
          Handle a timeout for non-blocking requests. This will call the user's defined error callback function
          (with args: (uuid_msgs.UniqueID, str)).

          :param event: regular rospy timer event object (not used)

          :param request_handler: a copy of the handler that gets bound when this callback is passed into the timer
          :type request_handler: :class:`.NonBlockingRequestHandler`

          :todo: respond on the error callback.
        '''
        already_handled = False
        self._lock.acquire()
        try:
            del self._request_handlers[request_handler.key]
        except KeyError:
            already_handled = True
        self._lock.release()
        if not already_handled:
            if request_handler.error_callback is not None:
                request_handler.error_callback(unique_id.toMsg(uuid.UUID(request_handler.key)), "timeout")

    def _internal_callback(self, msg):
        '''
          :param msg: message returned from the server (with pair id etc)
          :type msg: ServicePairResponse
        '''
        # Check if it is a blocking call that has requested it.
        key = unique_id.toHexString(msg.id)
        already_handled = False
        non_blocking_request_handler = None
        self._lock.acquire()
        try:
            request_handler = self._request_handlers[key]
            request_handler.response = msg.response
            if isinstance(request_handler, BlockingRequestHandler):
                request_handler.event.set()
                already_handled = True
            else:  # NonBlocking
                # make a copy and delete so we can release the lock. Process after.
                non_blocking_request_handler = request_handler.copy()
                del self._request_handlers[key]
        except KeyError:
            already_handled = True  # it's either a blocking, or a non-blocking call handled by the timeout
        self._lock.release()
        if not already_handled:
            # Could use EAFP approach here since they will almost never be None, but this is more readable
            if non_blocking_request_handler.callback is not None:
                request_handler.callback(msg.id, msg.response)
