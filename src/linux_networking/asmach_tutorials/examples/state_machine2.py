#!/usr/bin/env python

import roslib; roslib.load_manifest('asmach_tutorials')
import rospy
import asmach as smach
import smach_ros
from asmach import async

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    @async.function
    def execute_async(self, userdata):
        yield
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            async.returnValue('outcome1')
        else:
            async.returnValue('outcome2')


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    @async.function
    def execute_async(self, userdata):
        yield
        rospy.loginfo('Executing state BAR')
        async.returnValue('outcome1')
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = smach.async.run(sm.execute_async())



if __name__ == '__main__':
    main()
