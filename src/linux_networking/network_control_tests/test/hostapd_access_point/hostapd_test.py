#! /usr/bin/env python

import sys
import time
import subprocess
import unittest
import re

import roslib; roslib.load_manifest('network_control_tests')
import rospy
import rostest 

import dynamic_reconfigure.client
from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal
from ieee80211_channels.channels import IEEE80211_Channels

AP_REAL_IP = "192.168.68.1"
AP_FAKE_IP = "192.168.69.1"
STA_REAL_IP = "192.168.69.2"
STA_FAKE_IP = "192.168.68.2"

class HostapdTest(unittest.TestCase):
    def __init__(self, *args):
        super(HostapdTest, self).__init__(*args)
        rospy.init_node('hostapd_access_point_test')
        self.ap1_iface = rospy.get_param('~ap1_iface')
        self.ap2_iface = rospy.get_param('~ap2_iface')
        self.sta_iface = rospy.get_param('~sta_iface')

        self.dyn_ap1 = dynamic_reconfigure.client.Client("hostapd1")
        self.dyn_ap2 = dynamic_reconfigure.client.Client("hostapd2")
        self.reset_params = { 
            "enabled" : False,
            "ssid": "test",
            "wmm": False,
            "mode": 'b',
            "freq": 2412e6,
            "ieee80211n": False,
            "encryption_mode": "open",
            "encryption_pass": "",
            "txpower_auto": "True",
            "txpower": 0,
            "bitrate": 0 }
        self.dyn_ap1.update_configuration(self.reset_params)
        self.dyn_ap2.update_configuration(self.reset_params)

        self.hwsim_nat_setup_path = \
            roslib.packages.find_node('hostapd_access_point', 'hwsim_nat_setup.sh')

        self.srcnode = UdpmonsourceHandle('performance_test')
        self.srcnode.cancel_all_tests()

    def setUp(self):
        pass

    def tearDown(self):
        self.srcnode.cancel_all_tests()
        self.dyn_ap1.update_configuration(self.reset_params)
        self.dyn_ap2.update_configuration(self.reset_params)
        subprocess.call(["wpa_cli", "-i", self.sta_iface, "terminate"],
                        stdout = subprocess.PIPE, stderr = subprocess.STDOUT)

    def setup_nat_rules(self):
        p = subprocess.Popen([self.hwsim_nat_setup_path, 
                              self.ap1_iface, AP_REAL_IP, AP_FAKE_IP,
                              self.sta_iface, STA_REAL_IP, STA_FAKE_IP],
                             stdout = subprocess.PIPE, stderr = subprocess.STDOUT)
        (out, err) = p.communicate()
        self.assertEqual(p.returncode, 0,
                         "Setting hwsim NAT rules on %s and %s failed: %s"%
                         (self.ap1_iface, self.sta_iface, out))
        
    def start_wpa_supplicant(self, conffile):
        supp_conf_path = roslib.packages.find_resource("network_control_tests", conffile)
        supp_conf_path = supp_conf_path[0]
        ret = subprocess.call(["wpa_supplicant", "-Dnl80211", "-i" + self.sta_iface, "-B", "-c" + supp_conf_path],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(ret, 0,
                         "Starting wpa_supplicant with WEP conf on %s failed with ret code %d"%
                         (self.sta_iface, ret))

    def test_freq(self):
        freq = IEEE80211_Channels.get_freq(5, IEEE80211_Channels.BAND_2400_MHz)
        config = self.dyn_ap1.update_configuration({"enabled": True, "ssid": "testnet1",
                                                    "mode": 'b', "freq": freq})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(ap1_info.freq(), freq,
                         "Expected freq to be %ld, but instead it was %ld"%(freq, ap1_info.freq()))
        
        freq = IEEE80211_Channels.get_freq(44, IEEE80211_Channels.BAND_5000_MHz)
        config = self.dyn_ap1.update_configuration({"freq": freq})
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(config['status'], "FAIL",
                         "Expected setting an 802.11a freq when in 802.11b mode to fail: %s"%
                         (ap1_info.out))

        config = self.dyn_ap1.update_configuration({"freq": freq, "mode": "a", "enabled": True})
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(ap1_info.freq(), freq,
                         "Expected freq to be %ld, but instead it was %ld"%(freq, ap1_info.freq()))

    def test_txpower(self):
        # set txpower to 0dBm
        config = self.dyn_ap1.update_configuration({"enabled": True, "ssid": "testnet1",
                                                    "mode": 'b', "txpower_auto": False, "txpower": 0})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertEqual(config['txpower'], 0,
                         "Expected configuration txpower to be 0 instead it was %d"%(config['txpower']))
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(ap1_info.txpower(), 0,
                         "Expected iwconfig txpower to be 0 instead it was %d"%(ap1_info.txpower()))

        # set txpower to 10dBm
        config = self.dyn_ap1.update_configuration({"txpower": 10})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertEqual(config['txpower'], 10,
                         "Expected configuration txpower to be 10 instead it was %d"%(config['txpower']))
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(ap1_info.txpower(), 10,
                         "Expected iwconfig txpower to be 10 instead it was %d"%(ap1_info.txpower()))

        # set txpower to -10dBm, this is not legal so expecting to move to the nearest legal value, which is 0
        config = self.dyn_ap1.update_configuration({"txpower": -10})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertEqual(config['txpower'], 0,
                         "Expected configuration txpower to be 0 instead it was %d"%(config['txpower']))
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(ap1_info.txpower(), 0,
                         "Expected iwconfig txpower to be 0 instead it was %d"%(ap1_info.txpower()))

        # set txpower to 30dBm, this is not legal so expecting to move to the nearest legal value, which is 20
        config = self.dyn_ap1.update_configuration({"txpower": 30})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertEqual(config['txpower'], 20,
                         "Expected configuration txpower to be 20 instead it was %d"%(config['txpower']))
        ap1_info = IwconfigInfo(self.ap1_iface)
        self.assertEqual(ap1_info.txpower(), 20,
                         "Expected iwconfig txpower to be 20 instead it was %d"%(ap1_info.txpower()))


# just tests that legal values are accepted since there is no way to read the bitrate while in master mode
# and mac80211_hwsim does not actually limit rate (does not emulate rates)
    def test_bitrate(self):
        config = self.dyn_ap1.update_configuration({"enabled": True, "ssid": "testnet1",
                                                    "mode": 'g', "bitrate": 1*10**6})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertEqual(config['bitrate'], 1*10**6,
                         "Expected configuration bitrate to be %d instead it was %d"%
                         (1*10**6, config['bitrate']))

        config = self.dyn_ap1.update_configuration({"bitrate": 54*10**6})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertEqual(config['bitrate'], 54*10**6,
                         "Expected configuration bitrate to be %d instead it was %d"%
                         (54*10**6, config['bitrate']))

        config = self.dyn_ap1.update_configuration({"bitrate": 3*10**6})
        self.assertEqual(config['status'], "FAIL",
                         "Setting an illegal bitrate of 3Mbit/s should have failed")


    def test_txpower_auto(self):
        config = self.dyn_ap1.update_configuration({"enabled": True, "ssid": "testnet1",
                                                    "txpower_auto": True, "txpower": 0})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertTrue(config['txpower_auto'],
                        "Expected txpower_auto to be True")
        self.assertEqual(config['txpower'], 20,
                         "Expected txpower to be maximum (20) instead it was %d"%(config['txpower']))

    def test_start_stop(self):
        config = self.dyn_ap1.update_configuration({"enabled": True, "ssid": "testnet1"})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertTrue(config['enabled'],
                        "Expected hostapd to be running")
        for i in range(3):
            config = self.dyn_ap1.update_configuration({"enabled": False})
            self.assertEqual(config['status'], "OK",
                             "Operation failed: " + config['errmsg'])
            self.assertFalse(config['enabled'],
                             "Expected hostapd to be stopped")
            config = self.dyn_ap1.update_configuration({"enabled": True})
            self.assertEqual(config['status'], "OK",
                             "Operation failed: " + config['errmsg'])
            self.assertTrue(config['enabled'],
                            "Expected hostapd to be running")


    def test_multiple_aps(self):
        config = self.dyn_ap1.update_configuration({"enabled": True, "ssid": "testnet1"})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertTrue(config['enabled'],
                        "Expected hostapd to be running")
        config = self.dyn_ap2.update_configuration({"enabled": True, "ssid": "testnet2"})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])
        self.assertTrue(config['enabled'],
                        "Expected hostapd to be running")
        ret = subprocess.call(["ifconfig", self.sta_iface, "up"], 
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(ret, 0,
                         "ifup on interface %s failed with ret code %d"%
                         (self.sta_iface, ret))
        p = subprocess.Popen(["iwlist", self.sta_iface, "scan"], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        (out, err) = p.communicate()
        self.assertTrue(re.findall('testnet1', out),
                        "Expected testnet1 ssid to be found in station scan: " + out)
        self.assertTrue(re.findall('testnet2', out),
                        "Expected testnet2 ssid to be found in station scan: " + out)

    def test_ieee80211n(self):
        config = self.dyn_ap1.update_configuration({"ieee80211n": True, "enabled": True})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])        

# needs a newer hostapd version than available in Ubuntu 10.04 
# it has been tested with hostapd 0.7.3 which has wmm support 
    def disabled_test_wmm(self):
        config = self.dyn_ap1.update_configuration({"wmm": True, "enabled": True})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])        

    def test_encryption(self):
        #bring up AP in WPA mode (with wrong key)
        config = self.dyn_ap1.update_configuration({"ssid": "testnet1",
                                                    "enabled": True, "encryption_mode": "wpa",
                                                    "encryption_pass": "violetsareblue"})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])

        # setup NAT rules 
        self.setup_nat_rules()

        # starting wpa_supplicant on sta_interface in WEP mode
        self.start_wpa_supplicant("wpa_supplicant_wpa.conf")
        # give it some time to connect
        time.sleep(5.0)

        test = self.srcnode.create_test(bw = 2.0 * 10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = AP_FAKE_IP, sink_port = 12345,
                                        update_interval = 0.2)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.linkdown(),
                        "Link should be down since WPA pass is wrong")

        # set the correct key
        config = self.dyn_ap1.update_configuration({"encryption_pass": "rosesarered"})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])

        # setup NAT rules again to make sure ARP entries are in place
        self.setup_nat_rules()
        
        # give it some time to reconnect
        time.sleep(5.0)

        test = self.srcnode.create_test(bw = 2.0 * 10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = AP_FAKE_IP, sink_port = 12345,
                                        update_interval = 0.2)
        test.start()
        time.sleep(5.5)
        self.assertFalse(test.linkdown(),
                         "Link should be up since WPA pass is now correct")

        # stop wpa_supplicant
        subprocess.call(["wpa_cli", "-i", self.sta_iface, "terminate"],
                        stdout = subprocess.PIPE, stderr = subprocess.STDOUT)
        
        # starting wpa_supplicant on sta_interface in WPA2 mode
        self.start_wpa_supplicant("wpa_supplicant_wpa2.conf")
        
        # give it some time to reconnect
        time.sleep(5.0)

        test = self.srcnode.create_test(bw = 2.0 * 10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = AP_FAKE_IP, sink_port = 12345,
                                        update_interval = 0.2)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.linkdown(),
                         "Link should be down since client wants WPA2, but AP is in WPA mode")

        # set the correct key
        config = self.dyn_ap1.update_configuration({"encryption_mode": "wpa_wpa2"})
        self.assertEqual(config['status'], "OK",
                         "Operation failed: " + config['errmsg'])

        # setup NAT rules again to make sure ARP entries are in place
        self.setup_nat_rules()

        # give it some time to reconnect
        time.sleep(5.0)

        test = self.srcnode.create_test(bw = 2.0 * 10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = AP_FAKE_IP, sink_port = 12345,
                                        update_interval = 0.2)
        test.start()
        time.sleep(5.5)
        self.assertFalse(test.linkdown(),
                        "Link should be up because AP is in WPA-WPA2 mode, and STA in WPA2")


class IwconfigInfo:        
    def __init__(self, interface):
        p = subprocess.Popen(["iwconfig", interface], 
                             stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        (self.out, err) = p.communicate()
        if p.returncode != 0:
            raise IOError(p.returncode, "iwconfig on %s failed: %s"%(interface, err))

    def is_master(self):
        return re.findall('Mode:\s*(.aster)', self.out)

    def is_station(self):
        return re.findall('Mode:\s*(.anaged)', self.out)

    def freq(self):
        freq = re.findall('Frequency:\s*(\S+)\s*GHz', self.out)
        if not freq:
            raise IOError(-1, "Could not find frequency in iwconfig output: %s"%(self.out))
        return long(float(freq[0]) * 1e9)

    def txpower(self):
        txpower = re.findall('Tx-Power=\s*(\S+)\s*dBm', self.out)
        if not txpower:
            raise IOError(-1, "Could not find tx power in iwconfig output: %s"%(self.out))
        return int(txpower[0])

    def ap(self):
        if self.is_master():
            raise TypeError("Interface is in master mode, can't be associated to an AP")
        
        ap = re.findall('Access Point: \s*(\w\w:\w\w:\w\w:\w\w:\w\w:\w\w)', self.out)
        
        if ap:
            return ap[0]
        else:
            return None

    def associated(self):
        return self.ap() is not None
        
if __name__ == '__main__':
    try:
        rostest.run('network_control_tests', 'hostapd_test', HostapdTest)
    except KeyboardInterrupt, e:
        pass
