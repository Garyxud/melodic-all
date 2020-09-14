#! /usr/bin/env python

from __future__ import with_statement

import time
import sys
import threading
import socket
import struct
import bisect
import traceback

class MonitorSource :
    def __init__(self, latencybins, destaddr, rate, pkt_length, ros_returnpath = False, rostopic_prefix = "", roundtrip = False, max_return_time = 0.0, source_id = None, tos = 0, paused = False, sourceaddr = None):
        self.mutex = threading.Lock()
        self.outstanding = {}
        self.arrived = []
        self.latencybins = list(latencybins)
        self.latencybins.sort()
        self.latencybins.append(1e1000)
        self.interval = 1.0 / rate
        self.send_thread = threading.Thread(target = self.send_thread_entry, name = "udpmoncli: send_thread")
        self.recv_thread = threading.Thread(target = self.recv_thread_entry, name = "udpmoncli: recv_thread")
        self.dns_thread = threading.Thread(target = self.dns_thread_entry, name = "udpmoncli: dns_thread")
        self.pkt_length = pkt_length
        self.exiting = False
        self.sourceaddr = self.resolve_addr(sourceaddr)
        self.destaddr = destaddr
        self.exceptions = set()

        self.source_id = source_id
        self.roundtrip = roundtrip      
        self.ros_returnpath = ros_returnpath
        self.tos = tos

        self.max_return_time = max_return_time
        if roundtrip:
            self.max_rtt = self.latencybins[-2]
        else:
            self.max_rtt = self.latencybins[-2] + max_return_time

        self.sending_too_fast = False

        try:
            self.destaddr_ip = self.resolve_addr(self.destaddr)
        except:
            pass # We'll try again later if it fails.
        self.lost = 0
 
        self._reset_bins()
        
        if self.ros_returnpath:
            import rospy
            from msg import UdpSink
            self.sub = rospy.Subscriber(rostopic_prefix + "udpsink_feedback", UdpSink, self.receive_ros_feedback)
            while self.sub.get_num_connections() == 0 : # block until subscribing succeeds
                time.sleep(0.3)
                continue
            self.magic, = struct.unpack("=i", struct.pack("BBBB", 0xEF, 0x41, 0xC6, 0x34))
        else:
            self.magic, = struct.unpack("=i", struct.pack("BBBB", 0xEF, 0x41, 0xC6, 0x35))

        if self.source_id is not None:
            self.pktstruct = "=iddii"
        else:   
             self.pktstruct = "=iddi"
        self.hdr_len = struct.calcsize(self.pktstruct)
        if (pkt_length < self.hdr_len):
            print >> sys.stderr, "pkt_length must be at least", self.hdr_len
            return
        
        self.cv = threading.Condition()
        self.paused = True
        if not paused:
            self.start_monitor()

        self.window_start = time.time()
        self.dns_thread.start()
        self.recv_thread.start()
        self.send_thread.start()

    def receive_ros_feedback(self, msg):
        if msg.source_id != self.source_id:
            return 
        recv_time = time.time()        
        self.process_rcvd_packet(msg.send_time, msg.echo_time, recv_time, msg.seqnum)

    def get_source_id(self):
        return self.source_id

    def hit_send_rate_limit(self):
         sending_too_fast = self.sending_too_fast
         self.sending_too_fast = False
         return sending_too_fast

    def set_tx_bandwidth(self, bw):
        self.interval = (self.pkt_length * 8) / (bw + 0.00001)  

    def resolve_addr(self, addr):
        if addr == None:
            return addr
        host, port = addr
        return (socket.gethostbyname(host), port)

    def init_socket(self, sourceaddr = None):
        if sourceaddr != None and self.sourceaddr != sourceaddr:
            self.sourceaddr = self.resolve_addr(sourceaddr)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(0.2)
        if self.sourceaddr:
            self.socket.bind(self.sourceaddr)

        if self.tos:
            try:
                self.socket.setsockopt(socket.SOL_IP, socket.IP_TOS, self.tos)
            except socket.error, e:
                print "Could not set TOS:", str(e)

    def start_monitor(self, sourceaddr = None):
        print "Starting UDP monitor"
        if self.paused:
            self.init_socket(sourceaddr)
            with self.cv:
                self.paused = False
                self.cv.notify_all()

    def stop_monitor(self):
        if not self.paused:
            self.paused = True
            self.socket.close()

    def _reset_bins(self):
        self.bins = [0 for i in range(0,len(self.latencybins))]

    def shutdown(self):
        #print "Udp monitor starting to shut down"
        self.exiting = True
        with self.cv:
            self.cv.notify_all()

    # re-resolve the IP of the base station every 60 seconds.
    def dns_thread_entry(self):
        next_time = time.time()
        while not self.exiting:
            try:
                self.destaddr_ip = self.resolve_addr(self.destaddr)
                sys.stderr.write("Base station address is %s:%d\n"%self.destaddr_ip)
            except:
                sys.stderr.write("Failed to resolve base station address\n")
                continue 
            time.sleep(15)

    def send_thread_entry(self):
        next_time = time.time()
        seqnum = 0
        while not self.exiting:
            try:
                sleeptime = next_time - time.time()
                if (sleeptime < -1):
                    self.exceptions.add("Send thread too far behind. Resetting expectations.")
                    self.sending_too_fast = True
                    next_time = time.time()
                elif sleeptime > 0:
                    time.sleep(sleeptime)
                if self.paused:
                    with self.cv:
                        if not self.exiting:
                            self.cv.wait()
                    next_time = time.time()
                    continue
                seqnum = seqnum + 1
                next_time = next_time + self.interval
                send_time = time.time()
                if self.source_id is not None:
                    hdr = struct.pack(self.pktstruct, self.magic, send_time, 0, seqnum, self.source_id)
                else:
                    hdr = struct.pack(self.pktstruct, self.magic, send_time, 0, seqnum)
                with self.mutex:
                    self.outstanding[seqnum] = send_time
                self.socket.sendto(hdr.ljust(self.pkt_length), self.destaddr_ip)
            except Exception, e:
                self.exceptions.add(str(e))
                print "Got exception in send thread:", e
                #traceback.print_exc(10)
                #print self.destaddr
        #print "Udp monitor send_thread finished shut down"
        self.socket.close()

    def recv_thread_entry(self):
        if self.ros_returnpath:
            return
    
        while not self.exiting:
            if self.paused:
                with self.cv:
                    if not self.exiting:
                        self.cv.wait()
                next_time = time.time()
                continue
            try:
                indata = self.socket.recv(4096)
                recv_time = time.time()
                if self.source_id is None:
                    (magic, send_time, echo_time, seq_num) = struct.unpack(self.pktstruct, indata[0:self.hdr_len])
                else:
                    (magic, send_time, echo_time, seq_num, source_id) = struct.unpack(self.pktstruct, indata[0:self.hdr_len])
                if magic != self.magic or (self.source_id is not None and self.source_id != source_id) :
                    continue
                self.process_rcvd_packet(send_time, echo_time, recv_time, seq_num)
            except socket.timeout:
                pass
            except Exception, e:
                self.exceptions.add(str(e))
        #print "Udp monitor recv_thread finished shut down"

    def process_rcvd_packet(self, send_time, echo_time, recv_time, seq_num):
        if self.roundtrip:
            latency = recv_time - send_time
        else:
            latency = echo_time - send_time
            # print warnings for those packets that overran maximum round trip time 
            # due specifically to a large return time
            if recv_time - send_time > self.max_rtt and latency < self.latencybins[-2]:
                self.exceptions.add("Packet return time %.2fms exceeds maximum return time of %.2fms"%
                              ((recv_time - echo_time) * 1e3, self.max_return_time * 1e3))        
        self.bins[bisect.bisect(self.latencybins, latency)] += 1
        with self.mutex:
            self.arrived.append((send_time, latency, seq_num))
    
    def get_statistics(self):
        bins = [0 for i in range(0,len(self.latencybins))]
        with self.mutex: 
            now = time.time()
            arrived = self.arrived
            self.arrived = []
            outstanding = self.outstanding
            self.outstanding = {}
            exceptions = self.exceptions
            self.exceptions = set()
        window_end = now - self.max_rtt
        window_start = self.window_start
        sum_latency = 0.
        count = 0
        sum_latency_restricted = 0.
        count_restricted = 0

        missed = 0
        for pkt in arrived:
            (send_time, latency, seq_num) = pkt
            if send_time < window_end:
                count += 1
                sum_latency += latency
                bins[bisect.bisect(self.latencybins, latency)] += 1
                if seq_num in outstanding:
                    if latency <= self.latencybins[-2]:
                        count_restricted += 1
                        sum_latency_restricted += latency
                    elif send_time >= window_start:
                        missed += 1
                if seq_num in outstanding:
                    del outstanding[seq_num]
            else:
                self.arrived.append(pkt)

        for (seq_num, send_time) in outstanding.iteritems():
            if send_time < window_end:
                missed += 1
                self.lost += 1
            else:
                self.outstanding[seq_num] = send_time
        self.window_start = window_end
        return Statistics(count, count_restricted, missed, sum_latency, sum_latency_restricted, bins, window_start, window_end, exceptions)

    def get_smart_bins(self, window):
        stats = self.get_statistics()
        for s in stats.exceptions:
            print "Got exception", s
        return stats.get_smart_bins()

class Statistics:
    def __init__(self, count, count_restricted, missed, sum_latency, sum_latency_restricted, bins, window_start, window_end, exceptions):
        self.exceptions = exceptions
        self.count = count
        self.count_restricted = count_restricted
        self.missed = missed
        self.sum_latency = sum_latency
        self.sum_latency_restricted = sum_latency_restricted
        self.bins = bins
        self.window_start = window_start
        self.window_end = window_end

    def get_window_length(self):
        return self.window_end - self.window_start

    def get_percentage_bins(self):
        denom = self.count_restricted + self.missed
        if not denom: 
            denom = 1 # bins will be all zero anyhow in this case.
        denom = float(denom)
        return [ val / (denom) for val in self.bins ]
   
    def get_average_latency(self):
        if self.count: 
            return self.sum_latency / float(self.count)
        else:
            return 0.0

    def get_average_latency_restricted(self):
        if self.count_restricted: 
            return self.sum_latency_restricted / float(self.count_restricted)
        else:
            return 0.0

    def get_smart_bins(self):
        return self.get_percentage_bins(), self.get_average_latency(), self.get_average_latency_restricted()

    def accumulate(self, extra_stats):
        """Combines the stats from extra_stats into the existing stats.
        Expects the time windows to be adjacent."""
        if len(extra_stats.bins) != len(self.bins):
            raise ValueError("Tried to merge Statistics with different bin sizes.")
        
        if extra_stats.window_end == self.window_start:
            self.window_start = extra_stats.window_start
        elif extra_stats.window_start == self.window_end:
            self.window_end = extra_stats.window_end
        else:
            raise ValueError("Tried to accumulate non-adjacent Statistics.")
                
        for i in range(0, len(self.bins)):
            self.bins[i] += extra_stats.bins[i]

        self.count += extra_stats.count
        self.count_restricted += extra_stats.count_restricted
        self.missed += extra_stats.missed
        self.sum_latency += extra_stats.sum_latency
        self.sum_latency_restricted += extra_stats.sum_latency_restricted
        self.exceptions |= extra_stats.exceptions

class MonitorClient(MonitorSource):
    def __init__(self, latencybins, destaddr, rate, pkt_length, paused = False, sourceaddr = None):
        MonitorSource.__init__(self, latencybins, destaddr, rate, pkt_length, paused=paused, sourceaddr=sourceaddr, roundtrip = True)
        
  
if __name__ == "__main__":
    try:
        if not len(sys.argv) in [5, 6]:
            print "usage: udpmoncli.py <host> <port> <pkt_rate> <pkt_size> [<src_addr>]"
            sys.exit(1)
        host = sys.argv[1]
        port = int(sys.argv[2])
        rate = float(sys.argv[3])
        size = int(sys.argv[4])
        if len(sys.argv) == 6:
            src_addr = sys.argv[5]
        else:
            src_addr = '0.0.0.0' 
        #cli = MonitorClient([.005, .01, .1, .2, .3, .4, .5], (host, int(port)), rate, size)
        cli = MonitorClient([.005, .01, .025, .05, .075, .1], (host, int(port)), rate, size, sourceaddr = (src_addr, 0)) 
        try:
            display_interval = 0.5
            start_time = time.time()
            next_time = start_time
            while True:
                next_time = next_time + display_interval
                sleeptime = next_time - time.time()
                if sleeptime > 0:
                    time.sleep(sleeptime)
                if 0:
                    bins = cli.get_bins()
                else:
                    bins, average, average_restricted = cli.get_smart_bins(display_interval)
                print "%7.3f:"%(time.time() - start_time),
                for i in range(0,len(bins)):
                    print "%3i"%(int(100*bins[i])),
                    if i == 2:
                        print "  /",
                print "avg: %5.1f ms"%(1000*average), "avgr: %5.1f ms"%(1000*average_restricted), "loss: %6.2f %%"%(100 - 100 * sum(bins[0:-1]))
                sys.stdout.flush()
        finally:
            cli.shutdown()
            print >> sys.stderr, "Round trip latency summary (packets):" 
            for i in range(0, len(cli.latencybins)):
                print >> sys.stderr, "%.1f ms: %i before %i after"%(cli.latencybins[i] * 1000, sum(cli.bins[0:i+1]), sum(cli.bins[i+1:]) + cli.lost)

    except KeyboardInterrupt:
        print >> sys.stderr, "Exiting on CTRL+C."

