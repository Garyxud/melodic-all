# Grabbed this handy code from here: 
# http://efreedom.com/Question/1-1673483/Store-Callback-Methods

import weakref

class WeakCallback (object):
    """A Weak Callback object that will keep a reference to
    the connecting object with weakref semantics.

    This allows object A to pass a callback method to object S,
    without object S keeping A alive.
    """
    def __init__(self, mcallback):
        """Create a new Weak Callback calling the method @mcallback"""
        obj = mcallback.im_self
        attr = mcallback.im_func.__name__
        self.wref = weakref.ref(obj, self.object_deleted)
        self.callback_attr = attr
        self.token = None

    def __call__(self, *args, **kwargs):
        obj = self.wref()
        if obj:
            attr = getattr(obj, self.callback_attr)
            attr(*args, **kwargs)
        else:
            self.default_callback(*args, **kwargs)

    def default_callback(self, *args, **kwargs):
        """Called instead of callback when expired"""
        pass

    def object_deleted(self, wref):
        """Called when callback expires"""
        pass

class WeakCallbackCb(WeakCallback):
    def __init__(self, mcallback):
        WeakCallback.__init__(self, mcallback)
        self._deleted_cb = None

    def object_deleted(self, wref):
        if self._deleted_cb:
            self._deleted_cb(*self._args, **self._kwargs)
            # Typically, the _deleted_cb will have a reference to the
            # object that has us as a callback, so we need to break that
            # cycle after calling the callback.
            self.set_deleted_cb()

    def set_deleted_cb(self, cb = None, *args, **kwargs):
        self._deleted_cb = cb
        self._args = args
        self._kwargs = kwargs
        if not cb:
            self._args = None
            self._kwargs = None


