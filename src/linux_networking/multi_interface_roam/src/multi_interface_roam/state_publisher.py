#! /usr/bin/env python 

from __future__ import with_statement

import event
import copy

# TODO
# - Make CompositeStatePublisher so it can be garbage collected.
# - Add test for Unsubscribe exception, in particular case whene it is
# raised directly from subscribe.
# - Add test for CompositeStatePublisher.
# - Add test that mutable states get copied when stored into old_state.

class JustSubscribed:
    pass

class StatePublisher():
    def __init__(self, init_state):
        self._state = init_state
        self._event = event.Event()

    def get(self):
        return self._state

    def set(self, new_state):
        if self._state != new_state:
            old_state = self._state
            self._state = copy.deepcopy(new_state)
            self._event.trigger(old_state = old_state, new_state = new_state)
            self._setting_thread = None

    def subscribe(self, *args, **kwargs):
        f = args[0]
        args = args[1:]
        h = self._event.subscribe_repeating(f, *args, **kwargs)
        allkwargs = {'old_state' : JustSubscribed, 'new_state' : self._state}
        allkwargs.update(kwargs)
        try:
            f(*args, **allkwargs)
        except event.Unsubscribe:
            h.unsubscribe()
        return h

class CompositeStatePublisher(StatePublisher):
    def __init__(self, func, state_publishers):
        StatePublisher.__init__(self, None)
        n = len(state_publishers)
        self._func = func
        self._states = n * [ JustSubscribed ]
        self._has_been_set = n * [ False ]
        for i in range(0, n):
            state_publishers[i].subscribe(self._cb, i) 
   
    _set = StatePublisher.set

    def _cb(self, idx, old_state, new_state):
        if self._states[idx] != old_state:
            raise Exception("Unexpected state: %s != %s"%(self._states[idx], old_state))
        self._states[idx] = new_state
        all_set = all(self._has_been_set)
        if not all_set:
            self._has_been_set[idx] = True
            if all(self._has_been_set):
                self._has_been_set = [] # Silly optimization
                all_set = True
        if all_set:
            self._set(self._func(self._states))

    def set(self, new_state):
        raise Exception("set method called on CompositeStatePublisher")

if __name__ == "__main__":
    import unittest
    import sys
    
    def state_logger(l, old_state, new_state):
        l.append((old_state, new_state))

    class BasicTest(unittest.TestCase):
        def test_basic(self):
            s = StatePublisher(False)
            l = []
            h1 = s.subscribe(state_logger, l)
            h2 = s.subscribe(state_logger, l)
            s.set(1)
            h1.unsubscribe()
            s.set(2)
            h2.unsubscribe()
            self.assertEqual(l, [
                (JustSubscribed, False),
                (JustSubscribed, False),
                (False, 1),
                (False, 1),
                (1, 2),
                ])

        def test_composite(self):
            s1 = StatePublisher(0)
            s2 = StatePublisher(0)
            s = CompositeStatePublisher(sum, [s1, s2])
            l = []
            s.subscribe(state_logger, l)
            s1.set(1)
            s2.set(2)
            s1.set(5)
            self.assertEqual(l, [ (JustSubscribed, 0), (0, 1), (1, 3), (3, 7) ])

    if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
        import roslib; roslib.load_manifest('multi_interface_roam')
        import rostest
        rostest.unitrun('multi_interface_roam', 'state_publisher_basic', BasicTest)
    else:
        unittest.main()
