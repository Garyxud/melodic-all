#! /usr/bin/env python 
from twisted.internet.defer import Deferred, DeferredQueue, inlineCallbacks, returnValue
from twisted.internet import reactor
from twisted.internet.protocol import Protocol
from collections import deque
from weakcallback import WeakCallbackCb
import weakref
import sys
from event import Unsubscribe, Event

# FIXME Add test for ReadDescrEventStream

def async_sleep(t):
    d = Deferred()
    reactor.callLater(max(0, t), d.callback, None)
    return d

def event_queue(event):
    q = DeferredQueue()
    def cb(*args, **kwargs):
        q.put((args, kwargs))
    h = event.subscribe_repeating(cb)
    q.unsubscribe = h.unsubscribe
    return q

def now():
    d = Deferred()
    reactor.callLater(0, d.callback, None)
    return d

def wait_for_state(state, condition = None):
    d = Deferred()
    def cb(old_state, new_state):
        #print "wait_for_state, cb", old_state, new_state, condition 
        if condition is None or condition(new_state):
            #print "wait_for_state, cb, hit"
            d.callback(new_state)
            raise Unsubscribe
    state.subscribe(cb)
    return d

def wait_for_event(event, condition = None):
    d = Deferred()
    def cb(*args, **kwargs):
        if condition is None or condition(args, kwargs):
            d.callback(*args, **kwargs)
            raise Unsubscribe
    event.subscribe_repeating(cb)
    return d

class EventStream:
    """Event stream class to be used with select and switch."""
    def __init__(self, event = None):
        self._queue = deque()
        self._invoke_listener = None
        self.put = WeakCallbackCb(self._put)
        self.discard = False
        if event:
            event.subscribe_repeating(self.put)

    def _put(*args, **kwargs):
        """Puts an element into the queue, and notifies an eventual
        listener. Use pop rather than _pop or you will create cycles and
        prevent reference counts from going to zero."""
        self = args[0]
        if self.discard:
            return
        self._queue.append((args[1:], kwargs))
        if self._invoke_listener:
            self._trigger()

    def get(self):
        """Pops the next element off the queue."""
        return self._queue.popleft()
    
    def _trigger(self):
        """Used internally to to trigger the callback."""
        self._invoke_listener.callback(None)
        self._invoke_listener = None

    def listen(self):
        """Returns a Deferred that will be called the next time an event
        arrives, possibly immediately."""
        if self._invoke_listener:
            raise Exception("Event stream in use from multiple places simultaneously.")
        d = self._invoke_listener = Deferred()
        if self._queue:
            self._trigger()
        return d # Using d because self._trigger() may change self._invoke_listener

    def stop_listen(self):
        """Discards the Deferred set by listen."""
        assert self._invoke_listener or self._queue
        self._invoke_listener = None

    def set_discard(self, discard):
        """Tells the event stream whether it should discard all queued and
        incoming events."""
        self.discard = discard
        if discard:
            self._queue = deque()

class EventStreamFromDeferred(EventStream):
    def __init__(self, d = None):
        EventStream.__init__(self)
        if d is None:
            d = Deferred()
        self.deferred = d
        d.addCallback(self.put)

def StateCondition(*args, **kwargs):
    return EventStreamFromDeferred(wait_for_state(*args, **kwargs))

def EventCondition(*args, **kwargs):
    return EventStreamFromDeferred(wait_for_event(*args, **kwargs))

class Timeout(EventStream):
    def __init__(self, timeout):
        EventStream.__init__(self)
        h = reactor.callLater(max(0, timeout), self.put)
        def cancel():
            if not h.called:
                h.cancel()
        self.put.set_deleted_cb(cancel)

class Now(EventStream):
    def __init__(self):
        EventStream.__init__(self)
        reactor.callLater(0, self.put)

class ReadDescrEventStream(EventStream):
    def __init__(self, portType, *args, **kwargs):
        EventStream.__init__(self)
        put = self.put
        class _ReadDescrEventStreamProto(Protocol):
            def dataReceived(self, data):
                put(data)
        self.port = reactor.listenWith(portType, _ReadDescrEventStreamProto(), *args, **kwargs)
        def cancel(port):
            port.stopListening()
        self.put.set_deleted_cb(cancel, self.port)

    def recv(self):
        return self.get()[0][0]
    
@inlineCallbacks
def select(*events):
    """Listens to the provided EventStreams, and returns a set of integers
    indicating which ones have data ready, as soon as there its at least
    one that is ready."""
    ready_list = []
    done = Deferred()
    done_called = []
    def _select_cb(_, i):
        ready_list.append(i)
        if not done_called:
            # We don't do the callback directly or else cycles tend to form
            # which cause delays in releasing objects. 
            done_called.append(None)
            reactor.callLater(0, done.callback, None)
    for i in range(len(events)):
        events[i].listen().addCallback(_select_cb, i)
    try:
        yield done
    finally:
        for e in events:
            e.stop_listen()
        del events # Needed to avoid creating a cycle when events gets put
        del e      # into the traceback associated with returnValue.
    returnValue(ready_list)

@inlineCallbacks
def switch(cases, multiple = False):
    events, actions = zip(*cases.iteritems())

    ready_list = yield select(*events)

    for i in ready_list:
        args, kwargs = events[i].get()
        if actions[i]:
            actions[i](*args, **kwargs)
        if not multiple:
            break

def wrap_function(f):
    def run_f(g):
        def run_g(*args, **kwargs):
            return f(g, *args, **kwargs)
        return run_g
    return run_f

@wrap_function
def mainThreadCallback(f, *args, **kwargs):
    "Decorator that causes the function to be called in the main thread."
    reactor.callFromThread(f, *args, **kwargs)

@wrap_function
def async_test(f, *args, **kwargs):
    "Starts an asynchronous test, waits for it to complete, and returns its result."
    result = []
    def cb(value, good):
        result.append(good)
        result.append(value)
    inlineCallbacks(f)(*args, **kwargs).addCallbacks(callback = cb, callbackArgs = [True],
                                    errback  = cb, errbackArgs  = [False])
    while not result:
        reactor.iterate(0.02)
    if result[0]:
        # Uncomment the following line to check that all the tests
        # really are being run to completion.
        #raise(Exception("Success"))
        return result[1]
    else:
        result[1].printTraceback()
        result[1].raiseException()

def unittest_with_reactor(run_ros_tests):
    exitval = []
    def run_test():
        try:
            if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
                import roslib; roslib.load_manifest('multi_interface_roam')
                global rostest
                import rostest
                run_ros_tests()
            else:
                import unittest
                unittest.main()
            exitval.append(0)
        except SystemExit, v:
            exitval.append(v.code)
        except:
            import traceback
            traceback.print_exc()
        finally:
            reactor.stop()

    reactor.callWhenRunning(run_test)
    reactor.run()
    sys.exit(exitval[0])
    
if __name__ == "__main__":
    import unittest
    import sys
    import gc
    import threading
    #from twisted.internet.defer import setDebugging
    #setDebugging(True)

    class EventStreamTest(unittest.TestCase):
        def test_dies_despite_cb(self):
            """Test that EventStream gets unallocated despite its callback
            being held. As without another reference, calling the callback
            will have no effect."""
            es = EventStream()
            esr = weakref.ref(es)
            putter = es.put
            l = []
            putter.set_deleted_cb(lambda : l.append('deleted'))
            self.assertEqual(l, [])
            self.assertEqual(esr(), es)
            del es
            self.assertEqual(l, ['deleted'])
            self.assertEqual(esr(), None)

        @async_test
        def test_timeout_memory_frees_correctly(self):
            """Had a lot of subtle bugs getting select to free up properly.
            This test case is what pointed me at them."""
            before = 0
            after = 0

            # First few times through seems to create some new data, so
            # retry. (#4518)
            for iter in range(10):
                before = len(gc.get_objects())
                # The yielding statement seems necessary, some stuff only gets
                # cleaned up when going through the reactor main loop.
                yield select(Timeout(0.001))
                after = len(gc.get_objects())
                if before == after:
                    break
            self.assertEqual(before, after)
            
        @async_test
        def test_timeout_autocancel(self):
            self.assertEqual(len(reactor.getDelayedCalls()), 0)
            yield select(Timeout(0.3), Timeout(0.001))
            self.assertEqual(len(reactor.getDelayedCalls()), 0)
        
        @async_test
        def test_event_stream_from_event(self):
            e = Event()
            es = EventStream(e)
            e.trigger('hello world')
            yield select(es)
            self.assertEqual(es.get(), (('hello world',),{}))
            
    class SelectTest(unittest.TestCase):
        @async_test
        def test_select_param1(self):
            "Tests that the first parameter can get returned."
            self.assertEqual((yield (select(Timeout(.01), Timeout(100)))), [0])
        
        @async_test
        def test_select_param2(self):
            "Tests that the second parameter can get returned."
            self.assertEqual((yield (select(Timeout(100), Timeout(.01)))), [1])
        
        @async_test
        def test_select_both(self):
            "Tests that the both parameters can get returned."
            es1 = EventStream()
            es2 = EventStream()
            es1.put(None)
            es2.put(None)
            self.assertEqual((yield select(es1, es2)), [0, 1])
     
    class SwitchTest(unittest.TestCase):
        @async_test
        def test_switch_param(self):
            "Tests switch on single outcome."
            yield switch({
                Timeout(.01): lambda : None,
                Timeout(100): lambda : self.fail('Wrong switch'),
                })

        @async_test
        def test_switch_empty_action(self):
            "Tests that a None action doesn't cause an exception."
            yield switch({
                Timeout(.01): None,
                })

        @async_test
        def test_switch_both_single(self):
            "Tests switch on simultaneous, non-multiple."
            es1 = EventStream()
            es2 = EventStream()
            es1.put()
            es2.put()
            hits = []
            yield switch({
                es1: lambda : hits.append(None),
                es2: lambda : hits.append(None),
                    })
            self.assertEqual(len(hits), 1)
        
        @async_test
        def test_switch_both_multiple(self):
            "Tests switch on simultaneous, multiple."
            es1 = EventStreamFromDeferred()
            es2 = EventStreamFromDeferred()
            es1.deferred.callback(None)
            es2.deferred.callback(None)
            hits = []
            yield switch({
                es1: lambda _: hits.append(None),
                es2: lambda _: hits.append(None),
                    }, multiple = True)
            self.assertEqual(len(hits), 2)

        @async_test
        def test_switch_parameters(self):
            "Tests that switch passes parameters correctly."
            es = EventStream()
            es.put(3)
            es.put(4)
            yield switch({
                es: lambda v: self.assertEqual(v, 3),
                    }, multiple = True)
            yield switch({
                es: lambda v: self.assertEqual(v, 4),
                    }, multiple = True)
            self.assertRaises(IndexError, es._queue.pop)

    class DecoratorTest(unittest.TestCase):
        def test_main_thread_callback(self):
            "Tests that mainThreadCallback works."
            done = []
            @mainThreadCallback
            def cb(*args, **kwargs):
                done.append((args, kwargs))
            threading.Thread(target=cb, args=[1,2], kwargs={'a':'b'}).start()
            for i in range(0, 100):
                if done:
                    break
                reactor.iterate(0.02)
            self.assertEqual(done, [((1,2), {'a':'b'})])

    def run_ros_tests():
        rostest.unitrun('multi_interface_roam', 'eventstream', EventStreamTest)
        rostest.unitrun('multi_interface_roam', 'select', SelectTest)
        rostest.unitrun('multi_interface_roam', 'switch', SwitchTest)
        rostest.unitrun('multi_interface_roam', 'decorators', DecoratorTest)

    unittest_with_reactor(run_ros_tests)
