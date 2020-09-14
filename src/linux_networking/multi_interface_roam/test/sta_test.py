#! /usr/bin/env python

## This program does a test on an interface, as controlled by
## wpa_supplicant_node.

import roslib; roslib.load_manifest('multi_interface_roam')
import rospy
import multi_interface_roam.radio as radio
import random
import itertools
import time
import multi_interface_roam.async_helpers as async_helpers
import multi_interface_roam.mac_addr as mac_addr
import multi_interface_roam.system as system
import sys
import os
from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

class SuccessCounter:
    def __init__(self, name):
        self.name = name
        self.successes = 0
        self.tries = 0

    def failure(self):
        self.count(False)

    def success(self):
        self.count(True)

    def count(self, cond):
        self.tries += 1
        if cond:
            self.successes += 1

    def __str__(self):
        if self.tries:
            percent = 100.0 * self.successes / self.tries
        else:
            percent = 0
        return "%s: %i/%i (%.1f%%)"%(self.name, self.successes, self.tries, percent)

class Bss:
    def __init__(bssid, ssid, freq, ip):
        self.bssid = bssid
        self.ssid = ssid
        self.freq = freq
        self.ip = ip

class FastAssoc: # Associate just after unassociating.
    pass

class SlowAssoc: # Wait x seconds after unassociating.
    pass

class Reassociate: # Do not request unassociation.
    pass

class StaTester:
    def __init__(self, iface, bsses):
        self.bsses = bsses
        self.iface = iface
        self.radio = radio.Radio(iface)
        self.scan_counter = SuccessCounter('scn')
        self.mixed_scan_counter = SuccessCounter('mxsan')
        self.bgscan_counter = SuccessCounter('bgscn')
        self.ap_not_found_counter = SuccessCounter('AP_fnd')
        self.early_disassoc_counter = SuccessCounter("prsist")
        self.assoc_counters = { 
                FastAssoc:   SuccessCounter("fst"), 
                SlowAssoc:   SuccessCounter("slw"), 
                Reassociate: SuccessCounter("rasc"),
                }
        self.bss_counters = dict((bss['name'], SuccessCounter(bss['name'])) for bss in bsses)
        self.counters = self.assoc_counters.values() + \
                [self.early_disassoc_counter, self.mixed_scan_counter, self.bgscan_counter, self.scan_counter, self.ap_not_found_counter] + \
                self.bss_counters.values()

    @inlineCallbacks    
    def try_assoc(self, bss, method = None):
        # Already on this AP
        cur_assoc = self.radio.associated.get() 
        if cur_assoc:
            if mac_addr.same(cur_assoc.bssid, bss['bssid']):
                print >> sys.stderr, "Already associated to", bss['name']
                return
        del cur_assoc

        # Scan for this AP
        self.radio.scan([bss['freq']])
        pre_scan_assoc = self.radio.associated.get()
        print >> sys.stderr, "Scanning"
        scan_results = yield async_helpers.wait_for_event(self.radio.scan_results_event)
        post_scan_assoc = self.radio.associated.get()
        if pre_scan_assoc and post_scan_assoc:
            self.bgscan_counter.count(scan_results)
        elif not pre_scan_assoc and not post_scan_assoc:
            self.scan_counter.count(scan_results)
        else:
            self.mixed_scan_counter.count(scan_results)
        if not scan_results:
            print >> sys.stderr, "\nScan failed"
            return
        target_bss = filter(lambda cbss: mac_addr.same(cbss.bssid, bss['bssid']) and cbss.ssid == bss['ssid'], scan_results)
        self.ap_not_found_counter.count(target_bss)
        if not target_bss:
            print >> sys.stderr, "\nAP not found", bss['name']
            self.bss_counters[bss['name']].count(False)
            return

        # Decide which associate method to use
        if not self.radio.associated.get(): 
            method = SlowAssoc

        if method is None:
            # Randomly sample with priority to least sampled.
            r = random.random()
            universe = dict((m,c.tries) for m, c in self.assoc_counters.iteritems())
            most_visited = max(universe.itervalues())
            for m in universe:
                universe[m] = 2 + max(universe[m] - most_visited, 0)
            total = float(sum(universe.values()))
            for method in universe:
                r -= universe[method] / total
                if r < 0:
                    break

        print >> sys.stderr, "Associate method", method
        
        if method != Reassociate:
            self.radio.unassociate()
            print >> sys.stderr, "Unassociating"
            yield async_helpers.wait_for_state(self.radio.associated, lambda x: x == radio.Unassociated)

        if method == SlowAssoc:
            print >> sys.stderr, "Waiting after unassociate"
            yield async_helpers.async_sleep(3)

        # Associate
        self.radio.associate((bss['ssid'], bss['bssid']))
        print >> sys.stderr, "Associating"
        yield async_helpers.wait_for_state(self.radio.associated, lambda x: x != radio.Associating)

        # Log success
        assoc_state = self.radio.associated.get()
        self.assoc_counters[method].count(assoc_state)
        self.bss_counters[bss['name']].count(assoc_state)

        if self.radio.associated.get():
            # If association succeeded, make sure it is stable.
            print >> sys.stderr, "Waiting after associating", bss['name'], method
            yield async_helpers.async_sleep(1)
            self.early_disassoc_counter.count(self.radio.associated.get())
        else:
            print >> sys.stderr, "\nAssoc failed", bss['name'], method
            system.system('iwconfig')

    @inlineCallbacks
    def run_test(self, method = None):
        while not rospy.is_shutdown():
            random.shuffle(self.bsses)
            for bss in self.bsses:
                yield async_helpers.async_sleep(0.1)
                yield self.try_assoc(bss, method)
                #out = ["\r\033[2K"]
                out = []
                for counter in self.counters:
                    out.append(str(counter))
                print >> sys.stderr, " ".join(out)
                sys.stdout.flush()

def shutdown_by_ros(why):
    reactor.fireSystemEvent('shutdown')

def main():
    rospy.init_node("sta_tester")
    rospy.core.add_shutdown_hook(shutdown_by_ros)

    interface = rospy.get_param("~interface")
    bsses = rospy.get_param("~bsses")
    methodstr = rospy.get_param("~method", "None")
    try:
        method = {
                "FastAssoc" : FastAssoc,
                "SlowAssoc" : SlowAssoc,
                "Reassociate" : Reassociate,
                "None" : None,
                }[methodstr]
    except:
        rospy.logerr("Bad ~method value: %s", methodstr)
    else:
        reactor.callWhenRunning(StaTester(interface, bsses).run_test, method)
        reactor.run()

if __name__ == "__main__":
    main()
