#! /usr/bin/env python
            
import traceback
import sys

if False: # Set to true to use my implementation
    print "Using Blaise's implementation"

    class returnValue:
        def __init__(self, value = None):
            self.value = value
            raise self
    
    
    def function(func):
        def function_internal(*args, **kwargs):
            f = func(*args, **kwargs)
            m = f.send
            args = (None, )
            while True:
                try:
                    yield_val = m(*args)
                except StopIteration:
                    raise returnValue(None)
                else:
                    try:
                        m = f.send
                        if yield_val is None:
                            # We are yielding all the way back.
                            args = yield
                            args = (args, )
                        else:
                            # f is calling an asynchronous function.
                            m = f.send
                            r = yield_val
                            val = None
                            while True:
                                try:
                                    r.send(val)
                                    val = yield
                                except returnValue, r:
                                    args = (r.value, )
                                    break
                    except:
                        m = f.throw
                        args = list(sys.exc_info())
                        # Remove misleading lines from the trace.
                        try:
                            args[2] = args[2].tb_next.tb_next
                        except:
                            pass
        
        return function_internal
    
    def start(f):
        f.next()
        return f
    
    @function
    def print_result(f, name = "<unnamed>"):
        yield
        try:
            out = yield f
            print name, "returned:", out
        except:
            print name, "had exception:"
            tb = list(sys.exc_info())
            #tb[2] = tb[2].tb_next.tb_next
            traceback.print_exception(*tb)
    
    def run(f, name = "<unnamed>"):
        try:
            r = start(print_result(f, name))
            while True:
                r.send(None)
        except returnValue, r:
            pass

else: # Twisted implementation

    from twisted.internet import defer
    
    function = defer.inlineCallbacks
    returnValue = defer.returnValue

    #def function(func):
    #    @defer.inlineCallbacks
    #    def exception_grabber(*args, **kwargs):
    #        try:
    #            val = yield defer.inlineCallbacks(func)(*args, **kwargs)
    #        except returnValue:
    #            raise
    #        except:
    #            traceback.print_exc()
    #            raise
    #        returnValue(val)

    #    return exception_grabber


    @function
    def print_result(f, name = "<unnamed>"):
        #try:
            out = yield f
            print name, "returned:", out
        #except:
        #    print name, "had exception:"
        #    tb = list(sys.exc_info())
        #    #tb[2] = tb[2].tb_next.tb_next
        #    traceback.print_exception(*tb)
    
    def run(f, name = "<unnamed>"):
        try:
            r = print_result(f, name)
        except returnValue, r:
            pass

@function
def f(a, b):
    returnValue(a + b)
    yield

@function
def g2():
    raise Exception("Booh!")
    yield

@function
def g(a, b):
    yield g2()
    returnValue(5)

@function
def h(a, b):
    out = yield (g(a, b))
    returnValue(out)

@function
def i(a, b, c):
    out = yield (f(a, b))
    out2 = f(out, c)
    out = yield out2
    returnValue(out)

def main():
    def demo(s):
        run(eval(s), s)

    demo("f(1,2)")
    print
    sys.stdout.flush()
    demo("g(1,2)")
    print
    sys.stdout.flush()
    demo("h(1,2)")
    print
    sys.stdout.flush()
    demo("i(1,2,3)")
    sys.stdout.flush()

if __name__ == "__main__":
    main()
#    import trace
#    tracer = trace.Trace(ignoredirs = [sys.prefix, sys.exec_prefix])
#    tracer.run('main()')
