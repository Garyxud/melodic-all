#! /usr/bin/env python

import time
import unittest

import roslib; roslib.load_manifest('network_control_tests')
import rospy
import rostest 

import dynamic_reconfigure.client
from ieee80211_channels.channels import IEEE80211_Channels

class LinksysTest(unittest.TestCase):
    def __init__(self, *args):
        super(LinksysTest, self).__init__(*args)
        rospy.init_node('linksys_access_point_test')

        self.dyn_ap24 = dynamic_reconfigure.client.Client("linksys_2_4")
        self.dyn_ap5 = dynamic_reconfigure.client.Client("linksys_5")

    def test_linksys_24(self):
        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"enabled": False})
        rospy.loginfo("Disabling interface (while previously disabled) executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"ssid": "testnet", "mode": "g"})
        rospy.loginfo("Updating configuration (while disabled) executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertFalse(config['enabled'],
                         "Expected AP to be stopped")

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"enabled": True})
        rospy.loginfo("Enabling interface executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"ssid": "anotherssid", "encryption_mode": "wep", "encryption_pass": "0123456789",
                                                     "ieee80211n": True, "wmm": True})
        rospy.loginfo("Updating config (while enabled) executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"mode": "a", "ieee80211n": False})
        rospy.loginfo("Updating config (while enabled) with bad params executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "FAIL",
                        "Expected setting mode 802.11a on 2.4GHz interface to fail")

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"enabled": True, 
                                                     "mode": "b", "ieee80211n": False, "txpower": -5, "bitrate": 1000000})
        rospy.loginfo("Updating config (while enabled) including txpower executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertTrue(config['txpower'] >= 0,
                        "Expected txpower to be greater than 0 instead it was: %d dBm"%(config['txpower']))

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"bitrate": 11000000})
        rospy.loginfo("Updating just bitrate (while enabled) executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertTrue(config['bitrate'] == 11000000,
                        "Expected bitrate to be 11M instead it was: %d"%(config['bitrate']))

        start_time = time.time()
        config = self.dyn_ap24.update_configuration({"enabled": False})
        rospy.loginfo("Disabling interface (previously enabled) exeuted in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertFalse(config['enabled'],
                         "Expected AP to be stopped")

    def test_linksys_5(self):
        config = self.dyn_ap5.update_configuration({"enabled": False, "mode": "b", "ieee80211n": False, "freq": 0,
                                                    "bitrate": 0})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        start_time = time.time()
        config = self.dyn_ap5.update_configuration({"enabled": True})
        rospy.loginfo("Failed enable executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "FAIL",
                        "Expected enabling 5GHz interface while in default mode 'b' to fail" +
                        ", returned config: " + str(config))
        self.assertFalse(config['enabled'],
                         "Expected AP to be stopped")

        start_time = time.time()
        config = self.dyn_ap5.update_configuration({"enabled": True, "mode": "a", 
                                                    "freq": IEEE80211_Channels.get_freq(153, IEEE80211_Channels.BAND_5000_MHz)})
        rospy.loginfo("Enabling 5GHz executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertTrue(config['enabled'],
                         "Expected AP to be running")

        start_time = time.time()
        config = self.dyn_ap5.update_configuration({"freq": IEEE80211_Channels.get_freq(161, IEEE80211_Channels.BAND_5000_MHz)})
        rospy.loginfo("Change channel 5GHz executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        start_time = time.time()
        config = self.dyn_ap5.update_configuration({"ieee80211n": True, "bitrate": 0})
        rospy.loginfo("Enabling 802.11n executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        start_time = time.time()
        config = self.dyn_ap5.update_configuration({"enabled": False})
        rospy.loginfo("Disabling 5GHz executed in: %.2fs", time.time() - start_time)
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertFalse(config['enabled'],
                         "Expected AP to be stopped")

    def test_linksys_both(self):
        config = self.dyn_ap5.update_configuration({"enabled": True, "mode": "a", "bitrate": 0, "wmm": False,
                                                    "freq": IEEE80211_Channels.get_freq(153, IEEE80211_Channels.BAND_5000_MHz)})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        config = self.dyn_ap24.update_configuration({"enabled": True, "mode": "g", "ieee80211n": True, "bitrate": 0, "wmm": False,
                                                      "freq": IEEE80211_Channels.get_freq(9, IEEE80211_Channels.BAND_2400_MHz)})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        config = self.dyn_ap24.update_configuration({"ssid": "myssid24"})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        config = self.dyn_ap5.update_configuration({"ssid": "myssid5"})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])

        config = self.dyn_ap24.update_configuration({"txpower": 30})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        rospy.loginfo("Max txpower level on 2.4GHz is: %d dbm", config['txpower'])

        config = self.dyn_ap5.update_configuration({"txpower": -1})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        rospy.loginfo("Min txpower level on 5GHz is: %d dbm", config['txpower'])

    def test_linksys_security(self):
        config = self.dyn_ap5.update_configuration({"enabled": True, "mode": "a", "ieee80211n": False, "bitrate": 0, "wmm": False,
                                                    "freq": IEEE80211_Channels.get_freq(153, IEEE80211_Channels.BAND_5000_MHz),
                                                    "encryption_mode": "open", "encryption_pass": ""})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        config = self.dyn_ap24.update_configuration({"enabled": True, "mode": "g", "ieee80211n": False, "bitrate": 0, "wmm": False,
                                                     "freq": IEEE80211_Channels.get_freq(9, IEEE80211_Channels.BAND_2400_MHz),
                                                     "encryption_mode": "open", "encryption_pass": ""})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        config = self.dyn_ap24.update_configuration({"encryption_mode": "wpa2", "encryption_pass": "mypassword24"})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        config = self.dyn_ap5.update_configuration({"encryption_mode": "wep", "encryption_pass": "9876543210"})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        config = self.dyn_ap5.update_configuration({"wmm": True})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertTrue(config['encryption_mode'] == "wep",
                        "Wrong encryption mode: " + config['encryption_mode'])
        config = self.dyn_ap24.update_configuration({"wmm": True})
        self.assertTrue(config['status'] == "OK",
                        "Operation failed: " + config['errmsg'])
        self.assertTrue(config['encryption_mode'] == "wpa2",
                        "Wrong encryption mode: " + config['encryption_mode'])
                        
if __name__ == '__main__':
    try:
        rostest.run('network_control_tests', 'linksys_test', LinksysTest)
    except KeyboardInterrupt, e:
        pass
