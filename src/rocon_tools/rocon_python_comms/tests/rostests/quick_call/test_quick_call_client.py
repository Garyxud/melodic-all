#!/usr/bin/env python

""" Testing the service pair client """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs
import rocon_python_comms
import rostest
import unique_id
import threading


def generate_request_message():
    request = rocon_service_pair_msgs.TestiesRequest()
    request.data = "hello dude"
    return request

class TestServicePairClient(unittest.TestCase):
    '''
      Make timeouts of 3.0 here (c.f. the 2.0s sleep in the server)
      so that the first gets processed in time, but holds up the second. Note that the
      publisher callbacks that queue up get processed serially, not in parallel so that
      the second callback doesn't get done in time before timing out.
      
      If you make them both 2.0, then neither gets done in time.
    '''

    def __init__(self, *args):
        super(TestServicePairClient, self).__init__(*args)
        self.testies = rocon_python_comms.ServicePairClient('testies', rocon_service_pair_msgs.TestiesPair)
        self.threaded_testies = rocon_python_comms.ServicePairClient('threaded_testies', rocon_service_pair_msgs.TestiesPair)
        self.response_message = "I heard ya dude"
        self.non_threaded_event = threading.Event()
        self.threaded_event = threading.Event()
        self.non_threaded_response = None
        self.threaded_response = None
        rospy.sleep(0.5)  # rospy hack to give publishers time to setup
        
    def test_quick_call(self):
        thread = threading.Thread(target=self.send_separate_non_threaded_test)
        thread.start()
        response = self.testies(generate_request_message(), timeout=rospy.Duration(3.0))
        rospy.loginfo("Response [expecting 'None']: %s" % (response.data if response is not None else None))
        self.assertIsNone(response, "Response from the server is an invalid 'None'")
        if response is not None:
            self.assertEquals(response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, response.data))
        self.non_threaded_event.wait()
        self.assertIsNotNone(self.non_threaded_response, "Response from the server is an invalid 'None'")
        if self.non_threaded_response is not None:
            self.assertEquals(self.non_threaded_response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, self.non_threaded_response.data))
 
    def send_separate_non_threaded_test(self):
        '''
          Asserts in here don't really work. Yeah, they fire and throw exceptions if they fail their
          assertion, but they don't register back in the rostest framework. Ideally need to set a condition back in
          the test function that lets it read the result.
        '''
        self.non_threaded_response = self.testies(generate_request_message(), timeout=rospy.Duration(3.0))
        rospy.loginfo("Response [expecting result]: %s" % (self.non_threaded_response.data if self.non_threaded_response is not None else None))
        self.non_threaded_event.set()

    def test_threaded_quick_call(self):
        thread = threading.Thread(target=self.send_separate_threaded_test)
        thread.start()
        response = self.threaded_testies(generate_request_message(), timeout=rospy.Duration(3.0))
        rospy.loginfo("Response [expecting result]: %s" % (response.data if response is not None else None))
        self.assertIsNotNone(response, "Response from the server is an invalid 'None'")
        if response is not None:
            self.assertEquals(response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, response.data))
        self.threaded_event.wait()
        self.assertIsNotNone(self.threaded_response, "Response from the server is an invalid 'None'")
        if self.threaded_response is not None:
            self.assertEquals(self.threaded_response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, self.threaded_response.data))
  
    def send_separate_threaded_test(self):
        '''
          Asserts in here don't really work. Yeah, they fire and throw exceptions if they fail their
          assertion, but they don't register back in the rostest framework. Ideally need to set a condition back in
          the test function that lets it read the result.
        '''
        self.threaded_response = self.threaded_testies(generate_request_message(), timeout=rospy.Duration(3.0))
        rospy.loginfo("Response [expecting result]: %s" % (self.threaded_response.data if self.threaded_response is not None else None))
        self.threaded_event.set()
       
    def error_callback(self, error_message):
        """ User callback to pick up error messages. """

if __name__ == '__main__':
    rospy.init_node("test_service_proxy")
    rostest.rosrun('rocon_python_comms',
                   'test_service_pair_client',
                   TestServicePairClient) 