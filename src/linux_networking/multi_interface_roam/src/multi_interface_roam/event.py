#! /usr/bin/env python 

from __future__ import with_statement

import thread
import weakref

# TODO:
# - Add an exception that allows you to unsubscribe from a callback.

class MultipleUnsubscribe(Exception):
    pass

class Unsubscribe(Exception):
    pass

class _LivenessSensor():
    def __init__(self):
        self._cb = None

    def set_cb(self, cb = None, *args, **kwargs):
        self._cb = cb
        self._args = args
        self._kwargs = kwargs
        if not cb:
            self._args = None
            self._kwargs = None

    def __del__(self):
        if self._cb:
            self._cb(*self._args, **self._kwargs)

class EventCallbackHandle:
    def __init__(self, dispatcher):
        self._dispatcher = weakref.ref(dispatcher)
        self._auto_unsubscribed = False

    def unsubscribe(self):
        if not self._auto_unsubscribed:
            d = self._dispatcher()
            if d:
                d._unsubscribe(self)

    def _auto_unsubscribed():
        self._auto_unsubscribed = True
        del self._dispatcher

class _EventDispatcher:
    def __init__(self):
        self._subscribers = {}

    def _unsubscribe(self, h):
        try:
            del self._subscribers[h]
        except KeyError:
            raise MultipleUnsubscribe("Tried to unsubscribe from event more than once.")

    def _trigger(self, *args, **kwargs):
        for (h, (cb, firstargs, firstkwargs, repeating, _)) in self._subscribers.items():
            if not h in self._subscribers:
                continue # Don't call if deleted during triggering.
            # Prepare the positional parameters
            allargs = firstargs + args
            # Prepare the named parameters. Parameters given to
            # the subscribe call have precedence.
            allkwargs = dict(kwargs)
            allkwargs.update(firstkwargs)
            try:
                cb(*allargs, **allkwargs)
            except Unsubscribe:
                repeating = False
            except:
                import traceback
                traceback.print_exc()
            if not repeating:
                del self._subscribers[h]
                h._auto_unsubscribed = True

class Event:
    def __init__(self):
        self._liveness_sensor = _LivenessSensor()
        self._dispatcher = _EventDispatcher()
        self.trigger = self._dispatcher._trigger
        self.set_close_cb = self._liveness_sensor.set_cb
    
    def subscribe(self, cb, args, kwargs, repeating = True):
        """Subscribes to an event. 
        
        Can be called at any time and from any thread. Subscriptions that
        occur while an event is being triggered will not be called until
        the next time the event is triggered."""

        h = EventCallbackHandle(self._dispatcher)
        self._dispatcher._subscribers[h] = (cb, args, kwargs, repeating, self._liveness_sensor)
        return h
    
    def subscribe_once(*args, **kwargs):
        # We don't want the names we use to limit what the user can put in
        # kwargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], kwargs, repeating = False)

    def subscribe_repeating(*args, **kwargs):
        # We don't want the names we use to limit what the user can put in
        # kwargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], kwargs, repeating = True)
    
    def unsubscribe_all(self):
        for h in self._dispatcher._subscribers.keys():
            self._dispatcher._unsubscribe(h)

if __name__ == "__main__":
    import unittest
    import sys
        
    def append_cb(l, *args, **kwargs):
        l.append((args, kwargs))
            
    class Unsubscriber:
        def __init__(self, l, blocking):
            self.l = l
            if blocking:
                raise Exception("blocking unsubscribe has been removed")

        def cb(self, *args, **kwargs):
            append_cb(self.l, *args, **kwargs)
            self.h.unsubscribe()

    class BasicTest(unittest.TestCase):
        def test_basic(self):
            """Tests basic functionality.
            
            Adds a couple of callbacks. Makes sure they are called the
            right number of times. Checks that parameters are correct,
            including keyword arguments giving priority to subscribe over
            trigger."""
            e = Event()
            l1 = []
            l2 = []
            h1 = e.subscribe_repeating(append_cb, l1, 'd', e = 'f')
            e.subscribe_once(append_cb, l2, 'a', b = 'c')
            e.trigger('1', g = 'h')
            e.trigger('2', e = 'x')
            h1.unsubscribe()
            e.trigger('3')
            sys.stdout.flush()
            self.assertEqual(l1, [
                (('d', '1'), { 'e' : 'f', 'g' : 'h'}), 
                (('d', '2'), { 'e' : 'f'}),
                ])
            self.assertEqual(l2, [
                (('a', '1'), { 'b' : 'c', 'g': 'h'}),
                ])

#        def test_subscription_change(self):
#            """Test that the _subscription_change is called appropriately."""
#            l = []
#            class SubChangeEvent(Event):
#                def _subscription_change(self):
#                    l.append(len(self._subscribers))
#            e = SubChangeEvent()
#            h1 = e.subscribe_repeating(None)
#            h2 = e.subscribe_repeating(None)
#            h1.unsubscribe()
#            h3 = e.subscribe_repeating(None)
#            h2.unsubscribe()
#            h3.unsubscribe()
#            self.assertEqual(l, [0, 1, 2, 1, 2, 1, 0])

        def test_unsub_myself(self):
            """Tests that a callback can unsubscribe itself."""
            e = Event()
            l = []
            u = Unsubscriber(l, False)
            u.h = e.subscribe_repeating(u.cb)
            e.trigger('t1')
            e.trigger('t2')
            self.assertEqual(l, [
                (('t1',), {}),
                ])

        def test_multiple_unsubscribe_repeating(self):
            """Tests exceptoin on multiple unsubscribe for repeating subscribers."""
            e = Event()
            h = e.subscribe_repeating(None)
            h.unsubscribe()
            self.assertRaises(MultipleUnsubscribe, h.unsubscribe)

        def test_multiple_unsubscribe_once(self):
            """Tests exceptoin on multiple unsubscribe for non-repeating subscribers."""
            e = Event()
            h = e.subscribe_repeating(None)
            h.unsubscribe()
            self.assertRaises(MultipleUnsubscribe, h.unsubscribe)

        def test_unsubscribe_all(self):
            """Tests basic unsubscribe_all functionality."""
            e = Event()
            e.subscribe_repeating(None)
            e.subscribe_repeating(None)
            e.subscribe_repeating(None)
            e.subscribe_repeating(None)
            e.subscribe_repeating(None)
            e.unsubscribe_all()
            self.assertEqual(len(e._dispatcher._subscribers), 0)

#        def test_unsub_myself_blocking(self):
#            """Tests that a blocking unsubscribe on myself raises exception."""
#            e = Event()
#            l = []
#            u = Unsubscriber(l, True)
#            u.h = e.subscribe_repeating(u.cb)
#            self.assertRaises(DeadlockException, e.trigger, ['t1'])

        def test_unsub_myself_nonblocking(self):
            """Tests that a nonblocking unsubscribe on myself does not raise."""
            e = Event()
            l = []
            u = Unsubscriber(l, False)
            u.h = e.subscribe_repeating(u.cb)
            e.trigger('t1')
        
        def test_norun_sub_during_trig(self):
            """Tests that a callback that gets added during a trigger is
            not run."""
            e = Event()
            l = []
            def add_cb(iter):
                l.append(('add_cb', iter))
                e.subscribe_repeating(append_cb, l)
            h = e.subscribe_repeating(add_cb)
            e.trigger('v1')
            h.unsubscribe()
            e.trigger('v2')
            self.assertEqual(l, [('add_cb', 'v1'), (('v2', ), {})])
        
        def test_norun_unsub_during_trig(self):
            """Tests that a callback that gets deleted during a trigger is
            not run."""
            e = Event()
            l = []
            def rm_cb():
                l.append('rm')
                e.unsubscribe_all()
            e.subscribe_repeating(rm_cb)
            e.subscribe_repeating(rm_cb)
            e.trigger()
            self.assertEqual(l, ['rm'])

        def test_liveness_sensor_no_source(self):
            """Tests when the liveness sensor gets set off when the event
            disappears and there is no event source."""
            e = Event()
            l = []
            e.set_close_cb(l.append, 'closed')
            e.subscribe_repeating(lambda : None)
            self.assertEqual(l, [])
            del e
            self.assertEqual(l, ['closed'])

        def test_liveness_sensor_with_source(self):
            """Tests when the liveness sensor gets set off when the event
            disappears and there is an event source."""
            e = Event()
            t = e.trigger
            l = []
            e.set_close_cb(l.append, 'closed')
            h = e.subscribe_repeating(lambda : None)
            del e
            self.assertEqual(l, [])
            h.unsubscribe()
            self.assertEqual(l, ['closed'])

        def test_liveness_sensor_with_dropping_source(self):
            """Tests when the liveness sensor gets set off when the event
            disappears and there is an event source."""
            e = Event()
            t = e.trigger
            l = []
            e.set_close_cb(l.append, 'closed')
            h = e.subscribe_repeating(lambda : None)
            del e
            self.assertEqual(l, [])
            del t
            self.assertEqual(l, ['closed'])

    if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
        import roslib; roslib.load_manifest('multi_interface_roam')
        import rostest
        rostest.unitrun('multi_interface_roam', 'event_basic', BasicTest)
    else:
        unittest.main()
