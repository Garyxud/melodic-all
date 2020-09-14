import radio
import time
import state_publisher
import config
from twisted.internet import reactor
import mac_addr
import event
import multiset
from twisted.internet.defer import inlineCallbacks
import async_helpers
import random
import logging_config

scan_periods_log = logging_config.get_logger_stream_for_file('scan_periods')
known_bsses_log = logging_config.get_logger_stream_for_file('known_bsses')
radio_manager_decisions = logging_config.get_logger_stream_for_file('console.radio_manager_decisions')

def check_band(freq, bands):
    if freq < 3000 and bands & 1:
        return True
    if freq > 4000 and bands & 2:
        return True
    return False

class Frequency:
    def __init__(self, freq, period):
        self.frequency = freq
        self.period = period
        self.next_scan_time_by_iface = {}
        self.next_scan_time = 0
        self.do_not_retry_before = {}
        self.interfaces = multiset.Multiset()
        self.last_hit = 0

    def update_next_scan_time(self, iface, when):
        self.next_scan_time = when
        self.next_scan_time_by_iface[iface] = when

    def set_period(self, period):
        when = time.time() + period
        self.period = period
        self.next_scan_time = min(self.next_scan_time, when)
        for iface in self.next_scan_time_by_iface:
            self.next_scan_time_by_iface[iface] = min(self.next_scan_time_by_iface[iface], when)
    
class NoFrequenciesReady(Exception):
    def __init__(self, next_time):
        Exception.__init__(self)
        self.next_time = next_time

class FrequencyList:
    def __init__(self):
        self.frequencies = {}
        self.bands = 0
        self.scan_period_cold = config.get_parameter('scan_period_cold', 10)
        self.scan_reschedule_delay = config.get_parameter('scan_reschedule_delay', 2)
        self.scan_period_randomness = config.get_parameter('scan_period_randomness', 0)
        self.min_scan_interval = config.get_parameter('min_scan_interval', 1)

    def add(self, freq, iface):
        if freq not in self.frequencies:
            self.frequencies[freq] = Frequency(freq, self.scan_period_cold)
        self.frequencies[freq].interfaces.add(iface)

    def remove(self, freq, iface):
        self.frequencies[freq].interfaces.remove(iface)

    def next_scan_freq(self, iface, now = None, allow_early = False):
        if now == None:
            now = time.time()

        # Which frequencies can we consider?
        active_freqs = filter(lambda f: iface in f.interfaces and check_band(f.frequency, self.bands), self.frequencies.itervalues())
        
        # Pick the frequency with the next expiry time.
        try:
            freq = min(active_freqs, key = lambda f: f.next_scan_time)
        except ValueError: # No active frequencies
            raise NoFrequenciesReady(None)
        
        # If no frequency has expired, take into account when it was last seen by this interface.
        if freq.next_scan_time > now:
            #print "Pondering frequencies that have not been updated recently on ", iface.iface
            freq = min(active_freqs, key = lambda f: f.next_scan_time_by_iface.get(iface,0))
        
        earliest_scan_time = freq.do_not_retry_before.get(iface, 0)
        if not allow_early:
            earliest_scan_time = max(earliest_scan_time, freq.next_scan_time)
        if earliest_scan_time > now:
            raise NoFrequenciesReady(earliest_scan_time)
        rand_period = freq.period * (0.5 + self.scan_period_randomness * random.random())
        freq.update_next_scan_time(iface, now + rand_period)
        freq.do_not_retry_before[iface] = now + self.min_scan_interval
        return freq.frequency

    def next_scan_freqs(self, iface, count, allow_early = False):
        now = time.time()
        freqs = []
        for i in range(count):
            try:
                f = self.next_scan_freq(iface, now, allow_early)
            except NoFrequenciesReady, nfr:
                if freqs:
                    break
                raise
            freqs.append(f)
        return freqs

    def set_freq_period(self, freq, period):
        f = self.frequencies[freq]
        f.set_period(period)

    def reschedule(self, iface, freqs):
        when = time.time() + self.scan_reschedule_delay
        for f in freqs:
            self.frequencies[f].update_next_scan_time(iface, when)

    def hit(self, freq, stamp):
        """Called each time a bss is seen."""
        try:
            self.frequencies[freq].last_hit = max(self.frequencies[freq].last_hit, stamp)
        except KeyError:
            print "Got a scan on an unexpected frequency." # FIXME
            pass # We got a hit on a frequency that we aren't scanning for. Strange.

def make_id(bss):
    return (bss.ssid, bss.bssid)

class Bss:
    def __init__(self, bss):
        self.id = make_id(bss)
        self.ssid = bss.ssid
        self.bssid = bss.bssid
        self.by_iface = {}
        self.frequency = bss.frequency
    
    def update(self, bss, iface):
        assert bss.ssid == self.ssid
        assert bss.bssid == self.bssid
        if self.frequency != bss.frequency:
            print "Frequency for bss %s, %s has changed from %i to %i MHz."%(mac_addr.pretty(bss.bssid), bss.ssid, self.frequency, bss.frequency) # FIXME
        self.frequency = bss.frequency
        self.by_iface[iface] = bss

    def last_seen(self, iface = None):
        if iface is None:
            return max(bss.stamp.to_sec() for bss in self.by_iface.itervalues())
        if iface in self.by_iface:
            return self.by_iface[iface].stamp.to_sec()
        else:
            return 0

class BssList:
    def __init__(self, freq_list):
        self.bsses = {}
        self.freq_list = freq_list
        pass

    def update(self, bsses, iface):
        for bss in bsses:
            id = (bss.ssid, bss.bssid)
            if id not in self.bsses:
                self.bsses[id] = Bss(bss)
            self.bsses[id].update(bss, iface)
            self.freq_list.hit(bss.frequency, bss.stamp.to_sec())

        if True: # Gives an overview of currently known bsses.
            all = self.bsses.items()
            all.sort(key=lambda (x,y):(y.frequency, x))
            now = time.time()
            print >> known_bsses_log, "\033[2J\033[0;0H"
            print >> known_bsses_log, "Known BSSes bsses:"
            for _, bss in all:
                print >> known_bsses_log, mac_addr.pretty(bss.bssid), "%20.20s"%bss.ssid, bss.frequency,
                ifaces = bss.by_iface.keys()
                ifaces.sort()
                min_stamp = now - max(bss.by_iface.itervalues(), key = lambda bss: bss.stamp).stamp.to_sec()
                max_level = max(bss.by_iface.itervalues(), key = lambda bss: bss.level).level
                print >> known_bsses_log, "%5.1f/%3i"%(min_stamp,max_level),
                for iface in ifaces:
                    print >> known_bsses_log, iface.iface, "%5.1f/%3i"%(now - bss.by_iface[iface].stamp.to_sec(), bss.by_iface[iface].level),
                print >> known_bsses_log 
            print >> known_bsses_log 

            fl = self.freq_list.frequencies.keys()
            fl.sort()
            for f in fl:
                fc = self.freq_list.frequencies[f]
                if fc.last_hit:
                    print >> known_bsses_log, f, "%5.1f"%(now - fc.last_hit),
                else:
                    print >> known_bsses_log, f, "never",
                print >> known_bsses_log, "%5.1f"%(fc.next_scan_time - now),
                for iface in fc.next_scan_time_by_iface:
                    print >> known_bsses_log, "%s %5.1f"%(iface.iface, now - fc.next_scan_time_by_iface[iface]),
                print >> known_bsses_log
            print >> known_bsses_log

class ScanManager:
    def __init__(self):
        self.failed_scan_delay = config.get_parameter('failed_scan_delay', 1)
        self.scan_results = {}
        self.frequencies = FrequencyList()
        self.num_scan_frequencies = config.get_parameter('num_scan_frequencies', 4)
        self.scheduled_scan = {}
        self.bss_list = BssList(self.frequencies)
        self.new_scan_data = event.Event()

    def add_iface(self, iface):
        # Subscribe to state changes that might cause us to trigger a scan.
        iface.radio_sm.scanning.subscribe(self._scanning_state_cb, iface)
        iface.radio_sm.associated.subscribe(self._scanning_state_cb, iface)
        iface.radio_sm.associated.subscribe(self._log_associations, iface)
        iface.radio_sm.scanning_enabled.subscribe(self._scanning_state_cb, iface)
        
        # Subscribe to scan results
        iface.radio_sm.scan_results_event.subscribe_repeating(self._scan_event_cb, iface)
        # Subscribe to changes in frequency list
        iface.radio_sm.frequency_list.subscribe(self._frequency_list_cb, iface)

    def _scanning_state_cb(self, iface, old_state, new_state):
        """A state change that might cause us to start scanning has occurred."""
        self._trigger_scan(iface)

    def _log_associations(self, iface, old_state, new_state):
        if new_state:
            print >> radio_manager_decisions, "--> Associated to %s on %s."%(mac_addr.pretty(new_state.bssid), iface.iface)
        elif new_state == radio.Associating:
            print >> radio_manager_decisions, "--> Associating on %s."%iface.iface
        elif new_state == radio.Unassociated:
            print >> radio_manager_decisions, "--> Unassociated on %s."%iface.iface
        else: 
            print >> radio_manager_decisions, "--> ERROR!! Unknown association state %s on %s."%(new_state, iface.iface)

    @inlineCallbacks
    def _trigger_scan(self, iface):
        #if iface.radio_sm.associated.get():
        #    async_helpers.async_sleep(0.1)
        #print "_trigger_scan", iface.iface 
        if iface in self.scheduled_scan:
            if self.scheduled_scan[iface].active():
                self.scheduled_scan[iface].cancel()
            del self.scheduled_scan[iface]
        if not iface.radio_sm.scanning_enabled.get() or iface.radio_sm.scanning.get():
            #print "_trigger_scan bailing", iface.radio_sm.scanning_enabled.get(), iface.radio_sm.scanning.get()
            return # We are not in a scanning state, or we are currently scanning.
        try:
            #print "_trigger_scan association", iface.radio_sm.associated.get(), not iface.radio_sm.associated.get()
            #if iface.radio_sm.associated.get():
            #    return # FIXME Take this out once associated scanning works well.
            if iface.radio_sm.associated.get():
                freqs = self.frequencies.next_scan_freqs(iface, 1, False)
            else:
                freqs = self.frequencies.next_scan_freqs(iface, self.num_scan_frequencies, True)
        except NoFrequenciesReady, nfr:
            if nfr.next_time:
                self.scheduled_scan[iface] = reactor.callLater(max(0.1, nfr.next_time - time.time()), self._trigger_scan, iface)
                #print "No frequencies for ", iface.iface, nfr.next_time - time.time()
            #else:
                #print "No frequencies for", iface.iface
        else:
            print "Triggering scan", iface.iface, freqs
            rslt = yield iface.radio_sm.scan(freqs)
            if not rslt:
                print "Scan failed", iface.iface, freqs
                yield async_helpers.async_sleep(self.failed_scan_delay)
                self.frequencies.reschedule(iface, freqs)

    def _scan_event_cb(self, iface, bsses):
        #print "_scan_event_cb", len(bsses)
        self.bss_list.update(bsses, iface)
        self.new_scan_data.trigger()

    def _frequency_list_cb(self, iface, old_state, new_state):
        now = time.time()
        for f in new_state:
            self.frequencies.add(f, iface)
        
        if old_state != state_publisher.JustSubscribed:
            for f in old_state:
                self.frequencies.remove(f, iface)

        self._trigger_scan(iface)

class RadioManager:
    def __init__(self):
        self.scan_manager = ScanManager()
        self.best_active = None
        self.iface_associations = {}
        self.initial_inhibit_end = time.time() + config.get_parameter('initial_assoc_inhibit', 5)
        self.bss_expiry_time = config.get_parameter('bss_expiry_time', 5)
        self.activate_hysteresis = config.get_parameter('activate_hysteresis', 5)
        self.max_hot_frequencies = config.get_parameter('max_hot_frequencies', 3)
        self.scan_period_warm = config.get_parameter('scan_period_warm', 10)
        self.scan_period_hot = config.get_parameter('scan_period_hot', 4)
        self.reassociate_hysteresis = config.get_parameter('reassociate_hysteresis', 5)
        self.same_bss_penalty = config.get_parameter('same_bss_penalty', 20)
        self.interfaces = set()

        self.set_mode() # Initializes some variables
        
        self.scan_manager.new_scan_data.subscribe_repeating(self._new_scan_data)
        self.hot_bss_expiry_time = config.get_parameter('hot_bss_expiry_time', 5)
        self.warm_bss_expiry_time = config.get_parameter('warm_bss_expiry_time', 3 * self.scan_period_warm)

    def add_iface(self, iface):
        self.scan_manager.add_iface(iface)
        self.interfaces.add(iface)
        iface.radio_sm.associated.subscribe(self._associated_cb, iface)
        iface.dhcpdata.error_event.subscribe_repeating(self._dhcp_fail, iface)
    
    def set_mode(self, ssid="", bssid="", band=0, scan_only=False, iface = ""):
        self.scan_only = scan_only
        self.forced_ssid = ssid
        self.forced_bssid = bssid
        self.forced_band = band
        self.forced_iface = iface
        self.scan_manager.frequencies.bands = band
        for iface in self.interfaces:
            self._check_cur_bss(iface)
            self.scan_manager._trigger_scan(iface) # In case there were no frequencies before.
    
    def _associated_cb(self, iface, old_state, new_state):
        if new_state == radio.Unassociated:
            if iface in self.iface_associations:
                del self.iface_associations[iface]
        self._check_cur_bss(iface)

    def _check_cur_bss(self, iface):
        """Makes sure that the current bss satisfies forcing constraints."""
        #print "_check_cur_bss"
        cur_bss = iface.radio_sm.associated.get()
        if not cur_bss:
            return
        if not self.check_bss_matches_forcing(cur_bss):
            print iface.iface, "associated to", cur_bss.ssid, mac_addr.pretty(cur_bss.bssid), cur_bss.frequency
            print "but want", self.forced_ssid, self.forced_ssid, self.forced_band
            print "Unassociating because bss does not match requirements."
            iface.radio_sm.unassociate()
            return
        if not self.check_iface_matches_forcing(iface):
            print "Unassociating %s because interface forced to %s"%(iface, self.forced_iface)
            iface.radio_sm.unassociate()
            return

    def check_bss_matches_forcing(self, bss):
        """Checks that the bss matches the forced bssid, ssid and band."""
        if self.scan_only:
            return False
        if not check_band(bss.frequency, self.forced_band):
            return False
        if self.forced_bssid and self.forced_bssid != bss.bssid:
            return False
        if self.forced_ssid and self.forced_ssid != bss.ssid:
            return False
        return True

    def check_iface_matches_forcing(self, iface):
        if self.forced_iface and self.forced_iface != iface.iface:
            return False
        return True

    def _new_scan_data(self):
        print "_new_scan_data"
        #print "\033[2J"
        #print "\033[0;0H"
        now = time.time()
        if now < self.initial_inhibit_end:
            print "inhibited"
            return
        for iface in self.interfaces:
            if not self.check_iface_matches_forcing(iface):
                continue

            cur_assoc = iface.radio_sm.associated.get()
            #print "OK to try reassociating?", iface.iface, iface.radio_sm.scanning_enabled.get(), cur_assoc
            if cur_assoc and iface == self.best_active:
                continue

            if iface.radio_sm.scanning_enabled.get() and cur_assoc != radio.Associating:
                # Pick the best bss for this interface.
                candidate_bsses = filter(self.check_bss_matches_forcing, 
                        self.scan_manager.bss_list.bsses.itervalues())
                #print "Candidate bsses:", [mac_addr.pretty(bss.bssid) for bss in candidate_bsses]
                if candidate_bsses:
                    expiry_time = now - self.bss_expiry_time
                    best_bss = max(candidate_bsses, key = lambda bss: self.desirability(bss, expiry_time, iface))
                    if iface not in best_bss.by_iface or best_bss.by_iface[iface].stamp.to_sec() < expiry_time:
                        print "Best bss is expired.", mac_addr.pretty(best_bss.bssid)
                        best_bss = None
                else:
                    print "No candidate bsses."
                    best_bss = None
                if best_bss is None:
                    print "No candidate bsses or expired."
                    #print bool(candidate_bsses), [candidate_bsses]
                    #print self.scan_manager.bss_list.bsses.values()
                    continue

                best_desirability = self.desirability(best_bss, expiry_time, iface)
                print "Best bss:", mac_addr.pretty(best_bss.bssid), best_desirability

                if cur_assoc:
                    # Are we already associated to the best AP?
                    if make_id(cur_assoc) == best_bss.id:
                        print "Already associated to best bss."
                        continue

                    # Compute desirability for current association.
                    cur_id = make_id(cur_assoc)
                    if cur_id in self.scan_manager.bss_list.bsses:
                        cur_bss = self.scan_manager.bss_list.bsses[make_id(cur_assoc)]
                        cur_desirability = self.desirability(cur_bss, expiry_time, iface)
                    else:
                        cur_desirability = -1e1000
                    
                    # Is the best ap better enough than the current best?
                    if cur_desirability + self.reassociate_hysteresis > best_desirability:
                        print "Best bss not over hysteresis threshold: %s %f > %s %f"%(mac_addr.pretty(cur_assoc.bssid), cur_desirability, mac_addr.pretty(best_bss.bssid), self.desirability(best_bss))
                        continue

                # Let's associate
                print >> radio_manager_decisions, "Associating to %s (%f) on %s"%(mac_addr.pretty(best_bss.bssid), best_desirability, iface.iface),
                if cur_assoc:
                    print >> radio_manager_decisions, "from %s (%f)"%(mac_addr.pretty(cur_assoc.bssid), cur_desirability)
                else:
                    print >> radio_manager_decisions, "from unassociated"
                iface.radio_sm.associate_request.trigger(best_bss.id)
                self.iface_associations[iface] = best_bss.id
    
    def _dhcp_fail(self, iface):
        print >> radio_manager_decisions, "DHCP failed, taking down", iface.iface
        iface.interface_upper.restart()

    def _ping_fail():
        pass

    def update(self):
        """Called in the main update cycle after all the scores have been
        computed. Decides which interfaces should be activated."""

        def iface_score(iface):
            """Used to decide what to activate and disactivate."""
            return iface.prescore

        now = time.time()
        
        active = set()
        verified = set()
        for iface in self.interfaces:
            iface.active = iface.radio_sm.is_active.get()
            if iface.active:
                active.add(iface)
            if iface.ping_monitor.is_verified.get():
                verified.add(iface)
        inactive_verified = verified - active

        # Only keep one active interface.
        if active:
            self.best_active = max(active, key = iface_score)
        else:
            self.best_active = None
        if len(active) > 1:
            for iface in active:
                if iface != self.best_active:
                    iface.radio_sm.activate_request.set(False)
                    print >> radio_manager_decisions, "XXX Disactivating %s because %s is active and better."%(iface.iface, self.best_active.iface)

        # Activate a verified interface if it is better than the current
        # active interface.
        if inactive_verified:
            #print >> radio_manager_decisions, "Active interface selection."
            #for iface in verified:
            #    print >> radio_manager_decisions, iface.iface, iface_score(iface), iface in active
            best_inactive_verified = max(inactive_verified, key = iface_score)
            if not self.best_active or iface_score(best_inactive_verified) > iface_score(self.best_active) + self.activate_hysteresis:
                if not self.best_active:
                    print >> radio_manager_decisions, "XXX Activating %s because no current best active."%best_inactive_verified.iface
                else:
                    print >> radio_manager_decisions, "XXX Activating %s because it is better than %s."%(best_inactive_verified.iface, self.best_active.iface)
                best_inactive_verified.radio_sm.activate_request.set(True)

        # Keep a closer watch on the most relevant frequencies.
        #print self.scan_manager.bss_list.bsses
        #print [candidate_bsses]
        candidate_bsses = filter(self.check_bss_matches_forcing, 
                self.scan_manager.bss_list.bsses.itervalues())
        expiry_time = now - self.hot_bss_expiry_time
        candidate_bsses.sort(key = lambda bss: self.desirability(bss, expiry_time), reverse = True)
        #print [candidate_bsses]
        periods = dict((f, self.scan_manager.frequencies.scan_period_cold) 
                for f in self.scan_manager.frequencies.frequencies)
        hot_frequencies = 0
        print >> scan_periods_log, "\033[2J\033[0;0H"
        print >> scan_periods_log, "Candidate bsses:"
        for bss in candidate_bsses:
            print >> scan_periods_log, bss.ssid, mac_addr.pretty(bss.bssid), bss.frequency, self.desirability(bss, expiry_time), now - bss.last_seen()
            if hot_frequencies < self.max_hot_frequencies and periods[bss.frequency] != self.scan_period_hot:
                hot_frequencies += 1
                p = self.scan_period_hot
            elif bss.last_seen(iface) > now - self.warm_bss_expiry_time:
                p = self.scan_period_warm
            periods[bss.frequency] = min(periods[bss.frequency], p)
        print >> scan_periods_log, "Frequencies"
        #for f in self.scan_manager.frequencies.frequencies:
        freqs = [ f for f in self.scan_manager.frequencies.frequencies if check_band(f, self.forced_band) ]
        freqs.sort()
        for f in freqs:
            print >> scan_periods_log, f, periods[f], self.scan_manager.frequencies.frequencies[f].next_scan_time - now
            self.scan_manager.frequencies.set_freq_period(f, periods[f])

    def desirability(self, target_bss, expiry_time = 0, iface = None):
        #print "desirability", mac_addr.pretty(target_bss.bssid), 
        desirabilities = [bss.level for bss_iface, bss in target_bss.by_iface.iteritems() if bss.stamp.to_sec() > expiry_time and (iface is None or bss_iface == iface)]

        if desirabilities:
            # If we are calculating desirability for a particular interface,
            # penalize if another interface is already associating/associated
            # to the bss.
            penalty = 0
            if iface:
                target_id = make_id(target_bss)
                for i, id in self.iface_associations.iteritems():
                    if i != iface and id == target_id:
                        penalty = self.same_bss_penalty

            #print max(desirabilities)
            return max(desirabilities) - penalty
        else:
            #print -1e1000
            return -1e1000

