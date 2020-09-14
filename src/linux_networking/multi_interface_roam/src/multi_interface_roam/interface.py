#! /usr/bin/env python

import interface_upper
import ping_tester
import dhcp
from dhcp_apply_config import DhcpAddressSetter, DhcpRouteSetter, DhcpSourceRuleSetter
import config
from netlink_monitor import netlink_monitor, IFSTATE
import radio_sm
import traceback
import pythonwifi.iwlibs
import mac_addr
import socket
import state_publisher
import ipaddr

class Interface:
    def __init__(self, iface, tableid, cfgname):
        self.iface = iface
        self.cfgname = cfgname
        self.prettyname = config.get_interface_parameter(self.cfgname, 'name', self.cfgname)
        self.active = True
        self.priority = config.get_interface_parameter(self.cfgname, 'priority', 0)
        src_rule_setter = DhcpSourceRuleSetter(iface, tableid, tableid)
        self.tableid = str(tableid)
        base_station = config.get_parameter('base_station')
        ping_port = config.get_parameter('ping_port')
        self.ping_tester = ping_tester.PingTester(iface, 20, (base_station, ping_port), src_rule_setter.state_pub)

    def update(self, interval):
        self.diags = []
        self.status = netlink_monitor.get_status_publisher(self.iface).get()
       
        self._update_specialized()

        (bins, latency1, latency2) = self.ping_tester.update(interval)
        self.ping_latency = latency2
        self.ping_loss = 100 - 100 * bins[0]
        if self.status < IFSTATE.ADDR:
            self.goodness = self.status - IFSTATE.ADDR
            self.reliability = self.status - IFSTATE.ADDR
        else:
            self.goodness = 100 * bins[0] - latency2 # Goodness is how many packets made it through then average latency.

        if self.status < IFSTATE.LINK_ADDR:
            self.bssid = "NoLink"

class DhcpInterface(Interface):
    def __init__(self, iface, tableid):
        Interface.__init__(self, iface, tableid, iface)
        self.interface_upper = interface_upper.InterfaceUpper(iface)
        self.dhcpdata = dhcp.dhcp_client(iface)
        DhcpAddressSetter(iface, self.dhcpdata.binding_publisher)
        DhcpRouteSetter(iface, tableid, self.dhcpdata.binding_publisher)
        
        # FIXME RPFilter somewhere.
        # Flush ip rule on interface up somewhere.

class WiredInterface(DhcpInterface):
    def __init__(self, iface, tableid):
        DhcpInterface.__init__(self, iface, tableid)

    def _update_specialized(self):
        self.reliability = 100
        self.bssid = "Wired"

class WirelessInterface(DhcpInterface):
    def __init__(self, iface, tableid):
        DhcpInterface.__init__(self, iface, tableid)
        self.wifi = pythonwifi.iwlibs.Wireless(iface)
        self.iwinfo = pythonwifi.iwlibs.WirelessInfo(iface)
        self.radio_sm = radio_sm.radio_sm(iface)
        self.ping_monitor = ping_tester.PingMonitor(self)
        self.had_exception_last_time = False

    def _update_specialized(self):
        has_link = self.status > IFSTATE.LINK_ADDR
        had_exception_this_time = False
        try:
            self.essid = self.wifi.getEssid()
            self.diags.append(('ESSID', self.essid))
        except Exception, e:
            if has_link:
                had_exception_this_time = True
                if self.had_exception_last_time:
                    traceback.print_exc(10)
                    print
            self.diags.append(('ESSID', 'Error collecting data.'))
            self.essid = "###ERROR-COLLECTING-DATA###"

        try:
            self.bssid = mac_addr.pretty(self.wifi.getAPaddr())
            self.diags.append(('BSSID', self.bssid))
        except Exception, e:
            if has_link:
                had_exception_this_time = True
                if self.had_exception_last_time:
                    traceback.print_exc(10)
                    print
            self.bssid = "00:00:00:00:00:00"
            self.diags.append(('BSSID', 'Error collecting data.'))

        try:
            self.wifi_txpower = 10**(self.wifi.wireless_info.getTXPower().value/10.)
            self.diags.append(('TX Power (mW)', self.wifi_txpower))
            self.wifi_txpower = "%.1f mW"%self.wifi_txpower
        except Exception, e:
            if str(e).find("Operation not supported") == -1 and has_link:
                had_exception_this_time = True
                if self.had_exception_last_time:
                    traceback.print_exc(10)
                    print
            self.diags.append(('TX Power (mW)', 'Error collecting data.'))
            self.wifi_txpower = "unknown" 

        try:
            self.wifi_frequency = self.wifi.wireless_info.getFrequency().getFrequency()
            self.diags.append(('Frequency (Gz)', "%.4f"%(self.wifi_frequency/1e9)))
        except Exception, e:
            if has_link:
                had_exception_this_time = True
                if self.had_exception_last_time:
                    traceback.print_exc(10)
                    print
            self.wifi_frequency = 0
            self.diags.append(('Frequency', 'Error collecting data.'))

        got_stats = False
        if has_link:
            try:
                stat, qual, discard, missed_beacon = self.wifi.getStatistics()
                max_quality = self.wifi.getQualityMax().quality
                quality = qual.quality * 100 / max_quality
                self.diags.append(('Quality', quality))
                self.diags.append(('Signal (dB)', qual.siglevel))
                self.diags.append(('Noise (dB)', qual.nlevel))
                self.diags.append(('SNR (dB)', qual.siglevel - qual.nlevel))
                self.wifi_signal = qual.siglevel
                self.wifi_noise = qual.nlevel
                self.wifi_quality = quality
                self.reliability = quality
                got_stats = True
            except Exception, e:
                print "Error getting wireless stats on interface %s: %s"%(self.iface, str(e))
        
        if not got_stats:
            #print self.prettyname, "could not collect wireless data", e
            print
            self.reliability = 0
            self.wifi_quality = -1
            self.wifi_noise = 10000
            self.wifi_signal = -10000
            for s in [ 'Quality', 'Signal', 'Noise' ]:
                if has_link:
                    self.diags.append((s, 'Error collecting data.'))
                else:
                    self.diags.append((s, 'Unknown'))

        self.wifi_rate = None
        if has_link:
            try:
                self.wifi_rate = self.wifi.wireless_info.getBitrate().value
            except:
                pass
            if self.wifi_rate is None:
                try:
                    self.wifi_rate = self.iwinfo.getBitrate().value
                except:
                    pass
        if self.wifi_rate is not None:    
            self.diags.append(('TX Rate (Mbps)', self.wifi_rate / 1e6))
            self.wifi_rate = self.wifi._formatBitrate(self.wifi_rate)
        else:
            if has_link:
                print "Unable to determine TX rate on interface", self.iface
                self.diags.append(('TX Rate (Mbps)', 'Error collecting data.'))
            else:
                self.diags.append(('TX Rate (Mbps)', 'Unknown'))
            self.wifi_rate = "Unknown"

        self.had_exception_last_time = had_exception_this_time
        
class StaticRouteInterface(Interface):
    def __init__(self, route, tableid):
        gateway, iface = route.split('@')
        Interface.__init__(self, iface, tableid, route)
        self.gateway = socket.gethostbyname(gateway)
        self._fake_binding = state_publisher.StatePublisher(None)
        netlink_monitor.get_state_publisher(iface, IFSTATE.ADDR).subscribe(self._publish_binding)
        DhcpRouteSetter(iface, tableid, self._fake_binding)

    def _publish_binding(self, old_state, new_state):
        if new_state:
            ip, netbits = new_state
            net = ipaddr.IPv4Network("%s/%s"%(ip, netbits))
            self._binding = { "ip" : ip, "gateway" : self.gateway, "network_slashed" : "%s/%s"%(net.network, net.prefixlen) }
            self._fake_binding.set(self._binding)
        else:
            self._fake_binding.set(None)
    
    def _update_specialized(self):
        self.reliability = 100
        self.bssid = "Wired"


class NoType(Exception):
    pass

class UnknownType(Exception):
    pass

def construct(iface, tableid):
    try:
        type = config.get_interface_parameter(iface, 'type')
        if type == "wired":
            return WiredInterface(iface, tableid)
        if type == "wireless":
            return WirelessInterface(iface, tableid)
        if type == "static":
            return StaticRouteInterface(iface, tableid)
        raise UnknownType(type)
    except config.NoDefault:
        raise NoType()


if __name__ == "__main__":
    try:
        DhcpInterface('eth1', 50)
    except:
        import traceback
        traceback.print_exc()
    import twisted.internet.reactor as reactor
    reactor.run()
    import threading
    import time
    while True:    
        time.sleep(0.1)
        threads = threading.enumerate()
        non_daemon = sum(0 if t.daemon else 1 for t in threads)
        if non_daemon == 1:
            break
        print
        print "Remaining threads:", non_daemon, len(threads)
        for t in threads:
            print ("daemon: " if t.daemon else "regular:"), t.name
