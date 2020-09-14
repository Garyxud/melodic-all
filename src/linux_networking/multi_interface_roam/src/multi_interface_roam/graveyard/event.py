#! /usr/bin/env python

from __future__ import with_statement

import thread
import threading
import weakref

# TODO:
# - Add support for using a reactor instead of running directly.
# - Add an exception that allows you to unsubscribe from a callback.

class DeadlockException(Exception):
    pass

class MultipleUnsubscribe(Exception):
    pass
            
class ReentrantDetectingLockEnterHelper:
    def __init__(self, parent):
        self.parent = parent

    def __enter__(self):
        pass

    def __exit__(self, *args):
        return self.parent.release()

class ReentrantDetectingLock():
    def __init__(self, *args, **kwargs):
        self._lock = threading.Lock(*args, **kwargs)
        self._owner = None

    def acquire(self, blocking=1):
        ti = thread.get_ident()
        if self._owner == ti and blocking:
            raise DeadlockException("Tried to recursively lock a non recursive lock.")
        if self._lock.acquire(blocking):
            self._owner = ti
            return True
        else:
            return False

    def release(self):
        self._owner = None
        self._lock.release()

    def __enter__(self):
        self.acquire()

    def __exit__(self, *args):
        self.release()

    def __call__(self, msg):
        try:
            self.acquire()
            return ReentrantDetectingLockEnterHelper(self)
        except DeadlockException:
            raise DeadlockException(msg)

class EventCallbackHandle:
    def __init__(self, event, cb, args, kwargs, repeating):
        # Use a weakref to avoid creating a loop for the GC.
        self._event = weakref.ref(event)
        self._cb = cb
        self._args = args
        self._kwargs = kwargs
        self._call_lock = ReentrantDetectingLock()
        self._repeating = repeating

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.unsubscribe()

    def _trigger(self, args, kwargs):
        # Prepare the positional parameters
        allargs = self._args + args
    
        # Prepare the named parameters. Parameters given to
        # the subscribe call have precedence.
        allkwargs = dict(kwargs)
        allkwargs.update(self._kwargs)
    
        # Call the callback
        cb = self._cb
        with self._call_lock("Callback recursively triggered itself."):
            if cb is not None:
                if not self._repeating:
                    self.unsubscribe(blocking = False, _not_called_from_trigger = False)
                cb(*allargs, **allkwargs)
    
    def unsubscribe(self, blocking = True, _not_called_from_trigger = True):
        # Kill as many references as we can.
        # Clear _cb first as it is the one that gets checked in _trigger. 
        self._cb = None
        self._args = ()
        self._kwargs = ()
        event = self._event()
        if event is not None:
            try:
                del event._subscribers[self]
                event._subscription_change()
            except KeyError:
                # Don't check once callbacks because there is a race
                # between manual deletion and automatic deletion. If a once
                # callback is manually cleared, _repeating will be true, so
                # we will catch the second attempt.
                if self._repeating and _not_called_from_trigger:
                    raise MultipleUnsubscribe("Repeating callback unsubscribed multiple times.")
        # If not an automatic unsubscribe then set _repeating to true so
        # that the next unsubscribe can raise a MultipleUnsubscribe
        # exception. 
        if _not_called_from_trigger:
            self._repeating = True
        if blocking:
            # Wait until the _call_lock gets released.
            with self._call_lock("Callback tried to blocking unsubscribe itself."):
                pass

class Event:
    def __init__(self, name = "Unnamed Event"): 
        self._name = name
        self._subscribers = {}
        self._subscription_change()

    def subscribe(self, cb, args, kwargs, repeating = True):
        """Subscribes to an event. 
        
        Can be called at any time and from any thread. Subscriptions that
        occur while an event is being triggered will not be called until
        the next time the event is triggered."""

        h = EventCallbackHandle(self, cb, args, kwargs, repeating)
        self._subscribers[h] = None
        self._subscription_change()
        return h
    
    def subscribe_once(*args, **kwargs):
        # We don't want the names we use to limit what the user can put in
        # kwargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], kwargs, repeating = False)

    def subscribe_repeating(*args, **kwargs):
        # We don't want the names we use to limit what the user can put in
        # kwargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], kwargs, repeating = True)

    def trigger(*args, **kwargs):
        """Triggers an event.

        Concurrent triggers of a given callback are serialized using a lock, so 
        triggering from a callback will cause a deadlock."""
        
        self = args[0]
        args = args[1:]

        for h in self._subscribers.keys():
            h._trigger(args, kwargs)

    def _subscription_change(self):
        """Called at the end of each subscribe/unsubscribe. Can be
        overloaded in derived classes."""
        pass

    def unsubscribe_all(self, blocking = True):
        """Unsubscribes all subscribers that were present at the start of
        the call."""
        subs = self._subscribers.keys()
        for s in subs:
            s._repeating = False
            s.unsubscribe(blocking, _not_called_from_trigger = False)

if __name__ == "__main__":
    import unittest
    import sys
        
    def append_cb(l, *args, **kwargs):
        l.append((args, kwargs))
            
    class Unsubscriber:
        def __init__(self, l, blocking):
            self.l = l
            self.blocking = blocking

        def cb(self, *args, **kwargs):
            append_cb(self.l, *args, **kwargs)
            self.h.unsubscribe(blocking = self.blocking)

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

        def test_subscription_change(self):
            """Test that the _subscription_change is called appropriately."""
            l = []
            class SubChangeEvent(Event):
                def _subscription_change(self):
                    l.append(len(self._subscribers))
            e = SubChangeEvent()
            h1 = e.subscribe_repeating(None)
            h2 = e.subscribe_repeating(None)
            h1.unsubscribe()
            h3 = e.subscribe_repeating(None)
            h2.unsubscribe()
            h3.unsubscribe()
            self.assertEqual(l, [0, 1, 2, 1, 2, 1, 0])

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
            self.assertEqual(len(e._subscribers), 0)

        def test_unsub_myself_blocking(self):
            """Tests that a blocking unsubscribe on myself raises exception."""
            e = Event()
            l = []
            u = Unsubscriber(l, True)
            u.h = e.subscribe_repeating(u.cb)
            self.assertRaises(DeadlockException, e.trigger, ['t1'])

        def test_unsub_myself_nonblocking(self):
            """Tests that a nonblocking unsubscribe on myself does not raise."""
            e = Event()
            l = []
            u = Unsubscriber(l, False)
            u.h = e.subscribe_repeating(u.cb)
            e.trigger('t1')

    def wait_cv(cv, l, cb, trig):
        with cv:
            l.append((cb, trig, 'pre'))
            cv.notify_all()
            cv.wait()
            l.append((cb, trig, 'post'))

    class ThreadTest(unittest.TestCase):
        def setUp(self):
            self.e = Event()
            self.cv = threading.Condition()
            self.l = []
            self.h1 = self.e.subscribe_once(wait_cv, self.cv, self.l, 'cb1')
            self.h2 = self.e.subscribe_once(wait_cv, self.cv, self.l, 'cb1')
            self.t = threading.Thread(target = self.e.trigger, args=['t1'])
            self.t.start()

        def test_norun_sub_during_trig(self):
            """Tests that a callback that gets added during a trigger is
            not run."""
            
            # Trigger event
            with self.cv:
                # Handle first callback.
                while not self.l:
                    self.cv.wait()
                # This runs while wait_cv is waiting
                self.l.append('main')
                self.e.subscribe_repeating(append_cb, self.l, 'cb2')
                self.cv.notify_all()

                # Handle second wait_cv callback.
                while len(self.l) != 4:
                    self.cv.wait()
                self.l.append('main2')
                self.cv.notify_all()
            
            # Let the trigger finish
            self.t.join()

            self.expected = [
                ('cb1', 't1', 'pre'), 
                'main',
                ('cb1', 't1', 'post'), 
                ('cb1', 't1', 'pre'), 
                'main2',
                ('cb1', 't1', 'post'), 
                (('cb2', 't2'), {}),
                ]
            
            # Trigger event again
            self.e.trigger('t2')
            
            self.assertEqual(self.l, self.expected)

        def test_norun_unsub_during_trig(self):
            """Tests that a callback that gets deleted during a trigger is
            not run."""
            
            # Trigger event
            with self.cv:
                # Handle first callback.
                while not self.l:
                    self.cv.wait()
                # This runs while wait_cv is waiting
                self.l.append('main')
                unsubed = 0
                self.h1.unsubscribe(blocking = False)
                self.h2.unsubscribe(blocking = False)
                self.cv.notify_all()
            
            # Let the trigger finish
            self.t.join()

            self.expected = [
                ('cb1', 't1', 'pre'), 
                'main',
                ('cb1', 't1', 'post'), 
                ]

            # Trigger event again
            self.e.trigger('t2')
            
            self.assertEqual(self.l, self.expected)

    if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
        import roslib; roslib.load_manifest('multi_interface_roam')
        import rostest
        rostest.unitrun('multi_interface_roam', 'event_basic', BasicTest)
        rostest.unitrun('multi_interface_roam', 'event_thread', ThreadTest)
    else:
        unittest.main()
