#! /usr/bin/env python

import urllib, urllib2, base64
import string, re
import math

import roslib; roslib.load_manifest('linksys_access_point')
import rospy

import dynamic_reconfigure.server
from access_point_control.cfg import ApControlConfig
from ieee80211_channels.channels import IEEE80211_Channels

class LinksysApControl:

    def __init__(self, hostname, username, password, interface):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.interface = interface

        self.passmgr = urllib2.HTTPPasswordMgrWithDefaultRealm()
        self.passmgr.add_password(realm=None,
                             uri="http://%s/"%(hostname),
                             user=username,
                             passwd=password)
        self.auth_handler = urllib2.HTTPBasicAuthHandler(self.passmgr)
        self.opener = urllib2.build_opener(self.auth_handler)
        urllib2.install_opener(self.opener)

        self.current_config = {}
        self.get_current_config()

        self.set_req_args = {}
        self.set_req_args["action"] = "Apply"
        self.set_req_args["submit_type"] = ""
        self.set_req_args["change_action"] = ""
        self.set_req_args["commit"] = "1"
        self.set_req_args["next_page"] = ""

        self.avail_txpower_list = self.get_avail_txpower_list()

        node_name = rospy.get_name()
        for param_name in self.current_config:
            param_full_name = node_name + "/" + param_name
            if not rospy.has_param(param_full_name):
                rospy.set_param(param_full_name, self.current_config[param_name])

    def mw_to_dbm(self, mw):
        return int(10 * math.log10(float(mw)) + 0.5)

    def apply_request(self, args, page):
        req = urllib2.Request("http://%s/apply.cgi"%(self.hostname))

        req_args = dict(self.set_req_args) 
        if args:
            req_args.update(args)
        req_args["submit_button"] = page
        req.add_data(urllib.urlencode(req_args))

        auth = self.passmgr.find_user_password(None, "http://%s/apply.cgi"%(self.hostname))
        base64string = base64.encodestring("%s:%s" % auth)[:-1]
        req.add_header("Authorization", "Basic %s" % base64string)
        req.add_header("Referer", "http://%s/%s.asp"%(self.hostname, page))

        lines = urllib2.urlopen(req)
        html = lines.read()

        if string.find(html, "Invalid Value") != -1:
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] += "Invalid value for param in page %s"%(page)
        if string.find(html, "Settings are successful") == -1:
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] += "The request was not successful"

    def get_page_info(self, page):
        req = urllib2.Request("http://%s/%s.asp"%(self.hostname, page))
        auth = self.passmgr.find_user_password(None, "http://%s/%s.asp"%(self.hostname, page))
        base64string = base64.encodestring("%s:%s" % auth)[:-1]
        req.add_header("Authorization", "Basic %s" % base64string)

        lines = urllib2.urlopen(req)
        html = lines.read()

        return html

    def get_avail_txpower_list(self):
       html = self.get_page_info("Wireless_Advanced")
       out_list = re.findall("(?s)name=\"%s_txpwr.*?Array\((\"\d+\".*?)\)"%(self.interface), html)
       txpwr_list = re.findall("\"(\d+)\"", out_list[0])
       txpwr_list_mw = [ int(el) for el in txpwr_list ]
       return [ (self.mw_to_dbm(el), el) for el in txpwr_list_mw ]

    def get_wireless_basic_params(self):
        # mode & ssid & channel
        html = self.get_page_info("Wireless_Basic")
        
        mode_out = re.findall("(?s)name=\"%s_net_mode.*?_net_mode = '(.*?)'"%(self.interface), html)
        ssid_out = re.findall("value='(.*)' name=\"%s_ssid\""%(self.interface), html)
        if self.interface == "wl0":
            expression = "(?s)InitValue\(passForm.*?var ch = '(\d+)'"
        else:
            expression = "(?s)InitValue\(passForm.*?var ch_1 = '(\d+)'"
        channel_out = re.findall(expression, html)

        if (not mode_out) or (not ssid_out) or (not channel_out):
            raise Exception("Could not read interface " + self.interface + " mode or ssid or channel. " +
                            "Please check that the interface is set to Manual mode and not Wi-Fi Protected Setup")

        return mode_out[0], ssid_out[0], int(channel_out[0])

    def get_wireless_advanced_params(self):
        # bitrate & txpower
        html = self.get_page_info("Wireless_Advanced")

        bitrate_out = re.findall("(?s)name=\"%s_rate.*?value=\"(\d+)\" selected"%(self.interface), html)
        txpower_out = re.findall("(?s)name=\"%s_txpwr.*?wl_txpwr = '(\d+)'"%(self.interface), html)

        return int(bitrate_out[0]), int(txpower_out[0])

    def get_wireless_security_params(self):
        # encryption_mode | encryption_pass
        html = self.get_page_info("WL_WPATable")

        encryption_mode_out = re.findall("(?s)name=%s_security_mode.*?var security_mode = '(.*?)'"%(self.interface), html)

        if re.search("wpa2?_personal", encryption_mode_out[0]):
            encryption_mode = re.findall("(.*)_.*", encryption_mode_out[0])[0]
            encryption_pass = re.findall("name=%s_wpa_psk value='(.*?)'"%(self.interface), html)[0]
        elif re.search("wpa2?_enterprise", encryption_mode_out[0]):
            encryption_mode = re.findall("(.*)_.*", encryption_mode_out[0])[0] + "_enterprise"
            encryption_pass = re.findall("name=%s_radius_key value='(.*?)'"%(self.interface), html)[0]
        elif encryption_mode_out[0].find("wep") > -1:
            encryption_mode = ApControlConfig.ApControl_wep
            encryption_pass = re.findall("name=%s_key1 value='(.*?)'"%(self.interface), html)[0]
        else:
            encryption_mode = ApControlConfig.ApControl_open
            encryption_pass = ""

        return encryption_mode, encryption_pass

    def get_qos_params(self):
        # wmm
        html = self.get_page_info("QoS")

        wmm_out = re.findall("value=\"(.*?)\" name=wl_wme checked", html)

        if wmm_out[0] == "on":
            return True
        else:
            return False

    def get_current_config(self):
        mode, ssid, channel = self.get_wireless_basic_params()
        if self.interface == "wl0":
            band = IEEE80211_Channels.BAND_2400_MHz
        else:
            band = IEEE80211_Channels.BAND_5000_MHz
        
        self.current_config['ssid'] = ssid
        self.current_config['freq'] = float(IEEE80211_Channels.get_freq(channel, band))
        if mode == "disabled":
            self.current_config['enabled'] = False
        else:
            self.current_config['enabled'] = True
        if mode in ["a-only", "b-only", "g-only"]:
            self.current_config['mode'] = mode[0]
            self.current_config['ieee80211n'] = False
        elif mode == "bg-mixed":
            self.current_config['mode'] = "g"
            self.current_config['ieee80211n'] = False
        elif mode == "mixed" and band == IEEE80211_Channels.BAND_2400_MHz:
            self.current_config['mode'] = "g"
            self.current_config['ieee80211n'] = True
        elif mode == "mixed" and band == IEEE80211_Channels.BAND_5000_MHz:
            self.current_config['mode'] = "a"
            self.current_config['ieee80211n'] = True
        elif mode == "n-only":
            self.current_config['ieee80211n'] = True
        else:
            self.current_config['mode'] = "unknown"
            self.current_config['ieee80211n'] = False

        self.current_config['bitrate'], txpower = self.get_wireless_advanced_params()

        self.current_config['txpower_auto'] = False
        self.current_config['txpower'] = self.mw_to_dbm(txpower)

        self.current_config['encryption_mode'], self.current_config['encryption_pass'] = \
                self.get_wireless_security_params()

        self.current_config['wmm'] = self.get_qos_params()

    def set_wireless_basic(self, ssid, if_mode, channel):
        args = {}
        if ssid is not None:
            args["%s_ssid"%(self.interface)] = ssid
        args["%s_net_mode"%(self.interface)] = if_mode
        if channel is not None:
            args["%s_channel"%(self.interface)] = channel
        self.apply_request(args, "Wireless_Basic")

    def set_wireless_advanced(self, bitrate, txpower):
        args = {}
        args["%s_txpwr"%(self.interface)] = txpower
        args["%s_rate"%(self.interface)] = bitrate
        self.apply_request(args, "Wireless_Advanced")

    def set_wireless_security(self, encryption_mode, encryption_pass):
        args = {}

        if encryption_mode == ApControlConfig.ApControl_open:
            args["%s_security_mode"%(self.interface)] = "disabled"
        elif encryption_mode == ApControlConfig.ApControl_wep:
            args["%s_security_mode"%(self.interface)] = "wep"
            args["%s_WEP_key"%(self.interface)] = ""
            args["%s_wep"%(self.interface)] = "restricted"
            args["%s_key"%(self.interface)] = "1"
            args["%s_wep_bit"%(self.interface)] = "64" # or "128"
            args["%s_key1"%(self.interface)] = encryption_pass
        elif encryption_mode in [ApControlConfig.ApControl_wpa,\
                ApControlConfig.ApControl_wpa2]:
            if encryption_mode == ApControlConfig.ApControl_wpa:
                args["%s_security_mode"%(self.interface)] = "wpa_personal"
                args["%s_crypto"%(self.interface)] = "tkip" 
            elif encryption_mode == ApControlConfig.ApControl_wpa2:
                args["%s_security_mode"%(self.interface)] = "wpa2_personal"
                args["%s_crypto"%(self.interface)] = "tkip" 
            args["%s_wpa_gtk_rekey"%(self.interface)] = "3600"
            args["%s_wpa_psk"%(self.interface)] = encryption_pass
        elif encryption_mode in [ApControlConfig.ApControl_wpa_enterprise,\
                ApControlConfig.ApControl_wpa2_enterprise]:
            if encryption_mode == ApControlConfig.ApControl_wpa_enterprise:
                args["%s_security_mode"%(self.interface)] = "wpa_enterprise"
            else:
                args["%s_security_mode"%(self.interface)] = "wpa2_enterprise"
            args["%s_wpa_gtk_rekey"%(self.interface)] = "3600"
            args["%s_radius_key"%(self.interface)] = encryption_pass
        elif encryption_mode == ApControlConfig.ApControl_wpa_wpa2:
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] += "WPA & WPA2 encryption mode not supported"
            return
        self.apply_request(args, "WL_WPATable")

    def set_wmm(self, wmm):
        args = {}
        if wmm:
            args["wl_wme"] = "on"
        else:
            args["wl_wme"] = "off"
        self.apply_request(args, "QoS")

    def compare_configs(self, requested_config, read_config):
        if requested_config['enabled'] != read_config['enabled']:
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] += "Could not set enabled status, wrote %s, read %s"% \
                (requested_config['enabled'], read_config['enabled'])
            return
        
        if read_config['enabled']:
            for prop in ['mode', 'ssid', 'freq', 'ieee80211n', 'txpower', 'bitrate', 'wmm', 'encryption_mode']:
                if requested_config[prop] != read_config[prop]:
                    self.current_config['status'] = "FAIL"
                    self.current_config['errmsg'] += "Could not set %s, wrote %s, read %s"% \
                        (prop, str(requested_config[prop]), str(read_config[prop]))
            
            if read_config['encryption_mode'] != "open":
                if requested_config['encryption_pass'] != read_config['encryption_pass']:
                    self.current_config['status'] = "FAIL"
                    self.current_config['errmsg'] += "Could not set encryption pass, wrote %s, read %s"% \
                        (requested_config['encryption_pass'], read_config['encryption_pass'])
            
    def reconfigure(self, config, level):
        self.current_config['status'] = "OK"
        self.current_config['errmsg'] = ""

        change = False
        # enabled, ssid, freq, mode, ieee80211n
        if not config['enabled']:
            if self.current_config['enabled']:
                self.set_wireless_basic(None, "disabled", None)
                change = True
        else:
            if config['enabled'] != self.current_config['enabled'] or \
                    config['ssid'] != self.current_config['ssid'] or \
                    config['freq'] != self.current_config['freq'] or \
                    config['mode'] != self.current_config['mode'] or \
                    config['ieee80211n'] != self.current_config['ieee80211n']:
                new_channel = IEEE80211_Channels.get_channel(config['freq'])
                if not config['enabled']:
                    new_mode = "disabled"
                elif config["ieee80211n"]:
                    new_mode = "mixed"
                else: 
                    new_mode = config['mode'] + "-only"
                self.set_wireless_basic(config['ssid'], new_mode, new_channel)
                change = True

            # bitrate & txpower
            if config['txpower'] != self.current_config['txpower'] or \
                    config['bitrate'] != self.current_config['bitrate']:
                # find closest available tx power
                min_abs_diff = abs(config['txpower'] - self.avail_txpower_list[0][0])
                for i in range(0, len(self.avail_txpower_list)):
                    avail_power_dbm = self.avail_txpower_list[i][0]
                    if abs(config['txpower'] - avail_power_dbm) <= min_abs_diff:
                        min_abs_diff = abs(config['txpower'] - avail_power_dbm)
                        closest_power_idx = i
                config['txpower'] = self.avail_txpower_list[closest_power_idx][0]
                self.set_wireless_advanced(config["bitrate"], self.avail_txpower_list[closest_power_idx][1])
                change = True

            # wmm
            if config['wmm'] != self.current_config['wmm']:
                self.set_wmm(config['wmm'])
                change = True

            # security params:
            if config['encryption_mode'] != self.current_config['encryption_mode'] or \
                    (config['encryption_mode'] != "open" and 
                     config['encryption_pass'] != self.current_config['encryption_pass']):
                self.set_wireless_security(config['encryption_mode'], config['encryption_pass'])
                change = True

        # verify config
        if change:
            self.get_current_config()
            self.compare_configs(config, self.current_config) 

        if self.current_config['enabled']:
            return self.current_config
        else:
            config['status'] = self.current_config['status']
            config['errmsg'] = self.current_config['errmsg']
            config['enabled'] = False
            return config
      
if __name__ == "__main__":
    rospy.init_node("linksys_apcontrol_node")

    ip = rospy.get_param("~ip", "192.168.1.1") 
    user = rospy.get_param("~user", "") 
    password = rospy.get_param("~password", "admin") 
    interface = rospy.get_param("~interface", "wl0")

    ap = LinksysApControl(ip, user, password, interface)

    dynamic_reconfigure.server.Server(ApControlConfig, ap.reconfigure)

    rospy.spin()
