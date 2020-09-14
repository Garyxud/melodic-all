#! /usr/bin/env python

from __future__ import with_statement

import time
import sys

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy

from udpmoncli import MonitorSource

        
if __name__ == "__main__":
    from optparse import OptionParser
    try:
        usage = "usage: %prog [options] <host> <port> <pkt_rate> <pkt_size> "
        parser = OptionParser(usage=usage)
        parser.add_option("-a", "--address", dest="src_addr", default="0.0.0.0", help="source interface address")
        parser.add_option("-s", "--source_id", dest="source_id", type="int", default=None, help="MonitorSource id (use for multiple sources)")
        parser.add_option("-m", "--max_return_time", dest="max_return_time", type="float", default=0.0, help="Maximum time for return path (in sec)")
        parser.add_option("-R", dest="ros_returnpath", action="store_true", default=False, help="Use ROS instead of UDP for return path")
        parser.add_option("-1", dest="oneway", action="store_true", default=False, help="Measure one-way travel time instead of roundtrip")
        (options, args) = parser.parse_args(sys.argv[1:])
        
        if len(args) != 4:
            parser.print_help()
            sys.exit(1)

        host = args[0]
        port = int(args[1])
        rate = float(args[2])
        size = int(args[3])
    
        src_addr = options.src_addr
        source_id = options.source_id
        ros_returnpath = options.ros_returnpath
        oneway = options.oneway 
        max_return_time = options.max_return_time
        
        if ros_returnpath:
            if source_id is None:
                source_id = 1
            rospy.init_node("udpmonsource", anonymous=True)

        source = MonitorSource([.005, .01, .025, .05, .075, .1], (host, int(port)), rate, size, ros_returnpath = ros_returnpath, roundtrip=not oneway, source_id = source_id, sourceaddr = (src_addr, 0), max_return_time = max_return_time)

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
                    bins = source.get_bins()
                else:
                    bins, average, average_restricted = source.get_smart_bins(display_interval)
                print "%7.3f:"%(time.time() - start_time),
                for i in range(0,len(bins)):
                    print "%3i"%(int(100*bins[i])),
                    if i == 2:
                        print "  /",
                print "avg: %5.1f ms"%(1000*average), "avgr: %5.1f ms"%(1000*average_restricted), "loss: %6.2f %%"%(100 - 100 * sum(bins[0:-1]))
                sys.stdout.flush()
                if ros_returnpath and rospy.is_shutdown():
                    break
        finally:
            if oneway:
                print >> sys.stderr, "Oneway latency summary (packets):"
            else:
                print >> sys.stderr, "Round trip latency summary (packets):"
            for i in range(0, len(source.latencybins)):
                print >> sys.stderr, "%.1f ms: %i before %i after"%(source.latencybins[i] * 1000, sum(source.bins[0:i+1]), sum(source.bins[i+1:]) + source.lost)
            source.shutdown()

    except KeyboardInterrupt:
        print >> sys.stderr, "Exiting on CTRL+C."

