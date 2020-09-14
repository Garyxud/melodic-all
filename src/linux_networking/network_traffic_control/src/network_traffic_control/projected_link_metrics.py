#! /usr/bin/env python

import sys
import math

def get_projected_link_metrics(bandwidth_limit, latency, loss, packet_size, tx_bandwidth):
    bandwidth_limit = float(bandwidth_limit)
    tx_bandwidth = float(tx_bandwidth)
    if bandwidth_limit == 0.0:
        bandwidth_limit = 10000 * 1e6

    tx_bandwidth_loss_adjusted = tx_bandwidth * (1 - loss/100.0)

    link_saturated = tx_bandwidth_loss_adjusted > bandwidth_limit

    proj_bandwidth = min(bandwidth_limit, tx_bandwidth_loss_adjusted)
    
    loss_due_to_saturation = (1 - bandwidth_limit/tx_bandwidth) * 100.0

    proj_loss = max(loss_due_to_saturation, loss) 

    if not link_saturated:
        proj_latency = latency
    else:
        packet_send_time = (8.0 * packet_size) / bandwidth_limit
        netem_rule_in_place = (loss != 0.0 or latency != 0.0)

        if netem_rule_in_place:   
            queue_size_in_packets = math.ceil(latency/packet_send_time + 1)
            proj_latency = queue_size_in_packets * packet_send_time 
            
            # if extra loss is being added by netem, in addition to that caused by saturation
            # latency must be scaled accordingly
            if loss > loss_due_to_saturation:
                proj_latency = proj_latency * loss/(100.0 - loss_due_to_saturation)
        else: # i.e. just tbf, no netem
            proj_latency = packet_send_time
     
    return (proj_bandwidth, proj_latency, proj_loss)

if __name__ == '__main__':
    if len(sys.argv) != 6:
        print "Usage: projected_link_metrics.py <bandwidth_limit> <latency> <loss> <packet_size> <tx_bandwidth>\n"
        print "bandwidth_limit: simulated link capacity (in bps)"
        print "latency: simulated link latency (in seconds)"
        print "loss: packet loss (in %, but don't include the % sign)"
        print "packet_size: in bytes (NOT bits)"
        print "tx_bandwidth: send rate in bps"
        print 
        print "Example: projected_link_metrics.py 1000000 0.01 20.0 1500 1500000"
        print "Simulates a link capacity of 1Mbit/s with a 10ms latency and 20% packet loss, packet size of 1500bytes" \
            " and data sent at 1.5Mbit/s"
        exit(1)

    (bandwidth, latency, loss) = get_projected_link_metrics(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]),
                                                            int(sys.argv[4]), float(sys.argv[5]))

    print "Projected metrics: bandwidth %.2fKbit/s latency %.2fms loss %.2f%%"% \
        (bandwidth/1e3, latency*1e3, loss)

    
