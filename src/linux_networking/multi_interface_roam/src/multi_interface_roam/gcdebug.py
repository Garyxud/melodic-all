import inspect, gc
from collections import deque
import yapgvb
import os.path

def follow_back(a, n):
    """Handy function for seeing why an object is still live by walking
    back through its referrers."""
    def print_elem(e):
        print repr(e)[0:200]
        try:
            print e.f_code.co_filename, e.f_lineno
            #print e.f_locals
            #print dir(e)
        except:
            pass
    print
    print "Follow back:"
    print_elem(a)
    for i in range(n):
        r = gc.get_referrers(a)
        r.remove(inspect.currentframe())
        print
        print len(r)
        for e in r:
            print_elem(e)
        a = r[0]

def compare_before_after(before, after):
    """Handy function to compare live objects before and after some
    operation, to debug why memory is leaking."""
    beforeids = set(id(e) for e in before)
    afterids = set(id(e) for e in after)
    delta = afterids - beforeids - set([id(before)])
    for e in after:
        if id(e) in delta:
            print e

def dump_back_reference_graph(obj, maxdepth):
    obj = obj()
    if obj is None:
        print "Weakref was freed."
        return

    curframe = inspect.currentframe()
    
    todo = deque([(obj, 0)])
    strings = {}
    objects = {}
    depths = {}
    edges = []
    skipped = set()

    def element_string(e):
        if type(e) == list:
            return "list"
        if type(e) == type(curframe):
            return "frame: %s:%i"%(os.path.basename(e.f_code.co_filename), e.f_lineno)
        if type(e) == dict:
            return "\n".join(["%10s : %20s"%(str(k)[0:10], str(v)[0:10]) for k, v in e.iteritems()][0:10])
        
        #if type(e) == tuple:
        #    return "tuple"
        return str(e)[0:40]

    def list_str_bounded(l, join_str, max_indices, max_elem):
        l = [str(e)[0:max_elem] for e in l]
        if len(l) > max_indices:
            l = l[0:max_indices]
            l.append('...')
        return join_str.join(l)

    def edge_string(e1, e2):
        if type(e1) == list:
            return list_str_bounded([i for i in range(len(e1)) if e1[i] == e2], ", ", 10, 10)

        if type(e1) == dict:
            keys = [str(k)[0:20] for (k,v) in e1.iteritems() if e2 == v]
            return list_str_bounded([k for (k,v) in e1.iteritems() if e2 == v], "\n", 10, 20)
        
        return list_str_bounded([a for a in dir(e1) if e1.__getattribute__(a) == e2], "\n", 10, 20)

    def dont_trace(e):
        if type(e) == type(inspect):
            return True
        return False
    
    while todo:
        e, d = todo.popleft()
        ide = id(e)
        if ide in strings or d > maxdepth:
            continue
        strings[ide] = element_string(e)
        depths[ide] = d
        objects[ide] = e
        if dont_trace(e):
            skipped.add(ide)
            continue
        d += 1
        refs = gc.get_referrers(e)
        refs.remove(curframe)
        for r in list(refs):
            if r in todo or r == objects:
                refs.remove(r)
        todo.extend((r, d) for r in refs)
        edges.extend((id(r), ide) for r in refs)
        del refs
           
    print "Found %i nodes and %i edges"%(len(strings), len(edges))
    #for s in strings.values():
    #    print s

    graph = yapgvb.Digraph('Referrers')
    nodes = {}
    colors = ["red", "orange", "yellow", "green", "blue", "purple", "black"]
    ncol = len(colors)
    for (ids, s) in strings.items():
        nodes[ids] = graph.add_node(str(ids), label=s,
                color=colors[depths[ids]%ncol])
        if ids in skipped:
            nodes[ids].shape = 'box'
        if depths[ids] == maxdepth:
            nodes[ids].shape = 'parallelogram'
    
    for (id1, id2) in edges:
        if id1 in nodes and id2 in nodes:
            edge = nodes[id1] >> nodes[id2]
            s = edge_string(objects[id1], objects[id2])
            if s:
                edge.label = s

    graph.root = str(id(obj))
    #graph.layout(yapgvb.engines.twopi)
    graph.layout(yapgvb.engines.dot)
    graph.render('gcgraph.ps')
    del objects

if __name__ == "__main__":
    import weakref
    l1 = []
    l2 = [l1]
    #l2.append(l2)
    l3 = [l1, l2]
    l4 = [l3]
    l1.append(l4)
    class Foo:
        pass
    f = Foo()
    f.l = l1
    l4.append(f)
    wr = weakref.ref(f)
    del l1
    del l2
    del l3
    del l4
    del f
    dump_back_reference_graph(wr, 10)
