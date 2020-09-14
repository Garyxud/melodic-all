#! /usr/bin/env python

from __future__ import with_statement

from twisted.internet import reactor, interfaces
from zope.interface import implements
import scapy.all as scapy
import event
import socket
import sys

# Following code is an optimisation because scapy uses tcpdump to compile bpf filters. The code was pulled from 
# scapy and plopped down here.
import os
import struct
try:
    scapy_arch_linux_module = sys.modules['scapy.arch.linux']
except:
    scapy_arch_linux_module = sys.modules['scapy.arch']
_orig_attach_filter = scapy_arch_linux_module.attach_filter
_filter_cache = {}

def _attach_filter_replacement(s, filter):
    if not scapy_arch_linux_module.TCPDUMP:
        return

    try:
        nb, bpf = _filter_cache[filter]
    except KeyError:
        try:
            f = os.popen("%s -i %s -ddd -s 1600 '%s'" % (scapy.conf.prog.tcpdump,scapy.conf.iface,filter))
        except OSError,msg:
            log_interactive.warning("Failed to execute tcpdump: (%s)")
            return
        lines = f.readlines()
        if f.close():
            raise Scapy_Exception("Filter parse error")
        nb = int(lines[0])
        bpf = ""
        for l in lines[1:]:
            bpf += struct.pack("HBBI",*map(long,l.split()))

        _filter_cache[filter] = (nb, bpf)
    
    # XXX. Argl! We need to give the kernel a pointer on the BPF,
    # python object header seems to be 20 bytes. 36 bytes for x86 64bits arch.
    if scapy.arch.X86_64:
        bpfh = struct.pack("HL", nb, id(bpf)+36)
    else:
        bpfh = struct.pack("HI", nb, id(bpf)+20)  
    s.setsockopt(scapy_arch_linux_module.SOL_SOCKET, scapy_arch_linux_module.SO_ATTACH_FILTER, bpfh)

scapy_arch_linux_module.attach_filter = _attach_filter_replacement
# End of the horrid optimization

class AlreadyClosed(Exception):
    pass

class L2Port:
    implements(interfaces.IListeningPort)

    def __init__(self, proto, iface = 'any', filter = None, max_size = 9000, reactor = reactor):
        self._protocol = proto
        self._socket = scapy.L2Socket(iface, filter = filter).ins
        self._socket.setblocking(False)
        self._max_size = 9000
        self._protocol.makeConnection(self)
        self._reactor = reactor

    def fileno(self):
        if self._socket:
            return self._socket.fileno()
        else:
            return -1

    def doRead(self):
        try:
            data = self._socket.recv(self._max_size)
        except socket.error, e:
            if e.errno == 100:
                pass # This happens if the interface goes down.
        else:
            self._protocol.dataReceived(data)

    def startListening(self):    
        self._reactor.addReader(self)

    def stopListening(self):
        self._reactor.removeReader(self)
        self.connectionLost()

    def send(self, data):
        self._socket.send(data)
    
    def connectionLost(self, reason=None):
        self._socket = None

    def logPrefix(self):
        return "L2Port"

if __name__ == "__main__":
    import unittest
    import async_helpers
    from async_helpers import unittest_with_reactor, async_test
    from twisted.internet.defer import Deferred
    from twisted.internet.protocol import Protocol
    import random
    
    def tst_icmp_pkt():
        return str(
                scapy.Ether()/
                scapy.IP(src='127.0.0.1', dst='127.0.0.1')/
                scapy.ICMP(type='echo-request', seq=1, id=random.randint(0, 0xFFFF))
                )

    class L2PortTest(unittest.TestCase):
        @async_test
        def test_basic(self):
            deferred = Deferred()
            packet = tst_icmp_pkt()
            class TstProto(Protocol):
                def dataReceived(self, data):
                    if data == packet:
                        deferred.callback(None)

            proto = TstProto()
            port = reactor.listenWith(L2Port, proto, iface = 'lo', filter='icmp')
            port.send(packet)
            yield deferred
             
        @async_test
        def test_as_event_stream(self):
            es = async_helpers.ReadDescrEventStream(L2Port, iface = 'lo', filter='icmp')
            pkt = tst_icmp_pkt()
            es.port.send(pkt)
            while True:
                yield async_helpers.select(es)
                inpkt = es.recv()
                if inpkt == pkt:
                    break

    def run_ros_tests():
        rostest.unitrun('multi_interface_roam', 'pcap_descriptor', L2PortTest)
    
    unittest_with_reactor(run_ros_tests)
