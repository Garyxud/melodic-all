#! /usr/bin/env python

import urllib, urllib2, base64, httplib
import string, math, re, time

import roslib; roslib.load_manifest('ddwrt_access_point')
import rospy

import dynamic_reconfigure.server
from access_point_control.cfg import ApControlConfig
from ieee80211_channels.channels import IEEE80211_Channels

MAX_HTTP_REQUEST_TRIES=10
INTERVAL_BETWEEN_TRIES=1.0 # seconds

class IncompleteResponseBody(Exception):
    def __str__(self):
        return "Could not find </html> in response body"

class DdwrtApControl:

    def __init__(self, hostname, username, password, interface = "wl0"):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.interface = interface
        if interface == "wl0":
            self.other_interface = "wl1"
        else:
            self.other_interface = "wl0"

        self.passmgr = urllib2.HTTPPasswordMgrWithDefaultRealm()
        self.passmgr.add_password(realm=None,
                                  uri="http://%s/"%(hostname),
                                  user=username,
                                  passwd=password)
        self.auth_handler = urllib2.HTTPBasicAuthHandler(self.passmgr)
        self.opener = urllib2.build_opener(self.auth_handler)
        urllib2.install_opener(self.opener)

        # dd-wrt WRT610n specific config
        if self.interface == "wl0":
            self.band = IEEE80211_Channels.BAND_2400_MHz
        else:
            self.band = IEEE80211_Channels.BAND_5000_MHz
        self.min_txpower_mw = 1
        self.max_txpower_mw = 251

        self.current_config = {}
        self.get_current_config()

        node_name = rospy.get_name()
        for param_name in self.current_config:
            param_full_name = node_name + "/" + param_name
            if not rospy.has_param(param_full_name):
                rospy.set_param(param_full_name, self.current_config[param_name])

    def mw_to_dbm(self, mw):
        return int(10 * math.log10(float(mw)) + 0.5)

    def dbm_to_mw(self, dbm):
        return int(math.pow(10, float(dbm) / 10.0) + 0.5)

    def send_http_request(self, req):
        count = 0
        
        while True:
            count += 1
            try:
                resp = urllib2.urlopen(req, timeout = 5.0)
                body = resp.read()
                if not re.findall("</html>", body):
                    raise IncompleteResponseBody()
                break
            except (urllib2.URLError, httplib.BadStatusLine, IncompleteResponseBody), e:    
                if count < MAX_HTTP_REQUEST_TRIES:
                    rospy.logwarn("HTTP request failed attempting again: %s", str(e))
                    time.sleep(INTERVAL_BETWEEN_TRIES)
                    continue
                else:
                    raise e

        return body

    def apply_wrt_request(self, req_args, page):
        req = urllib2.Request("http://%s/apply.cgi"%(self.hostname))

        req.add_data(urllib.urlencode(req_args))

        auth = self.passmgr.find_user_password(None, "http://%s/apply.cgi"%(self.hostname))
        base64string = base64.encodestring("%s:%s" % auth)[:-1]
        req.add_header("Authorization", "Basic %s" % base64string)
        req.add_header("Referer", "http://%s/%s.asp"%(self.hostname, page))

        try:
            self.send_http_request(req)
        except (urllib2.URLError, httplib.BadStatusLine, IncompleteResponseBody), e:
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] += "HTTP request failed: " + str(e)

    def get_page_info(self, page):        
        req = urllib2.Request("http://%s/%s.asp"%(self.hostname, page))
        auth = self.passmgr.find_user_password(None, "http://%s/%s.asp"%(self.hostname, page))
        base64string = base64.encodestring("%s:%s" % auth)[:-1]
        req.add_header("Authorization", "Basic %s" % base64string)

        return self.send_http_request(req)

    def find_ssid(self, interface, html):
        ssid = re.findall(r"<input[^>]*name=\"%s_ssid\"[^>]*value=[^>]*\"([^>]+?)\"[^>]*>"%(interface), html)
        if not ssid:
            return None
        return ssid[0]

    def find_mode(self, interface, html, mode):
        mode_block = re.findall(r"(?s)<select[^>]*name=\"%s_%s\"(.*?)</select>"%(interface, mode), html)
        if not mode_block:
            return None
        mode_block = mode_block[0]
        mode = re.findall(r"(?s)<option[^>]*value=[^\">]*?\"([^>]+?)[\\\"][^>]*?selected", mode_block)
        if not mode:
            return None
        return mode[0]

    def find_channel(self, interface, html):
        channel = re.findall(r"var %s_channel[^;]*?'(\d+)'"%(interface), html)
        if not channel:
            return None
        return int(channel[0])

    def find_bitrate(self, interface, html):
        bitrate_block = re.findall(r"(?s)<select[^>]*name=\"%s_rate\"(.*?)</select>"%(interface), html)
        if not bitrate_block:
            return None
        bitrate_block = bitrate_block[0]
        bitrate = re.findall(r"(?s)<option[^>]*value=[^\">]*?\"([^>]+?)[\\\"][^>]*?selected", bitrate_block)
        if not bitrate:
            return 0
        return int(bitrate[0])

    def find_txpower(self, interface, html):
        txpower = re.findall(r"<input[^>]*name=\"%s_txpwr\"[^>]*value=[^>]*\"([^>]+?)\"[^>]*>"%(interface), html)
        if not txpower:
            return None
        return int(txpower[0])

    def find_wmm(self, interface, html):
        wmm = re.findall(r"name=\"%s_wme\"[^>*]value=\"(.*?)\"[^>*]checked=\"checked\""%(self.interface), html)
        if not wmm:
            return None
        if wmm[0] == "on":
            return True
        elif wmm[0] == "off":
            return False
        else:
            return None

    def find_wep_key(self, interface, html):
        wep_key = re.findall(r"<input[^>]*name=[^>]*%s_key1[^>]*value=[^>]*\"([^>]*?)\"[^>]*>"%(interface), html)
        if not wep_key:
            return None
        return wep_key[0]

    def find_wpa_psk(self, interface, html):
        wpa_psk = re.findall(r"<input[^>]*name=[^>]*%s_wpa_psk[^>]*value=[^>]*\"([^>]*?)\"[^>]*>"%(interface), html)
        if not wpa_psk:
            return None
        return wpa_psk[0]

    def find_txpower(self, interface, html):
        txpower = re.findall(r"<input[^>]*name=\"%s_txpwr\"[^>]*value=[^>]*\"([^>]+?)\"[^>]*>"%(interface), html)
        if not txpower:
            return None
        return int(txpower[0])

    def get_other_interfaces_args(self):
        extra_args = {}
        html = self.get_page_info("Wireless_Basic")
        interface_list = re.findall(r"<select[^>]*name=[^>]*\"([a-zA-Z0-9]+)_mode\"[^>]*>", html)

        for interface in interface_list:
            if interface != self.interface:
                ssid = self.find_ssid(interface, html)
                mode = self.find_mode(interface, html, "mode")
                if mode is None:
                    mode = "ap"
                if ssid is not None:
                    extra_args["%s_mode"%(interface)] = mode
                    extra_args["%s_ssid"%(interface)] = ssid

        return extra_args

    def get_wireless_basic_params(self):
        # mode & ssid & channel
        html = self.get_page_info("Wireless_Basic")
        
        ssid = self.find_ssid(self.interface, html)
        mode = self.find_mode(self.interface, html, "net_mode")
        if mode is None:
            mode = "disabled"
        channel = self.find_channel(self.interface, html)
        
        return mode, ssid, channel

    def get_wireless_advanced_params(self):
        # bitrate & txpower & wmm
        html = self.get_page_info("Wireless_Advanced-%s"%(self.interface))

        bitrate = self.find_bitrate(self.interface, html)
        txpower = self.find_txpower(self.interface, html)
        wmm = self.find_wmm(self.interface, html)
        
        return bitrate, txpower, wmm

    def get_wireless_security_params(self):
        # encryption_mode | encryption_pass
        html = self.get_page_info("WL_WPATable")

        enc = self.find_mode(self.interface, html, "security_mode")
        if not enc:
            encryption_mode_out=['disabled']
        else:
            encryption_mode_out=[enc]

        if enc.find("psk") > -1:
            if enc == "psk":
                enc_mode = ApControlConfig.ApControl_wpa
            elif enc == "psk2":
                enc_mode = ApControlConfig.ApControl_wpa2
            elif enc == "psk psk2":
                enc_mode = ApControlConfig.ApControl_wpa_wpa2
            else:
                return None, ""
            enc_pass = self.find_wpa_psk(self.interface, html)
        elif enc == "wep":
            enc_mode = ApControlConfig.ApControl_wep
            enc_pass = self.find_wep_key(self.interface, html)
        elif enc == "disabled":
            enc_mode = ApControlConfig.ApControl_open
            enc_pass = ""
        else:
            return None, ""

        return enc_mode, enc_pass

    def get_current_config(self):
        mode, ssid, channel = self.get_wireless_basic_params()
        
        if mode is None:
            raise Exception("Could not read interface %s mode"%(self.interface))
        if mode != "disabled" and ssid is None:
            raise Exception("Could not read interface %s ssid"%(self.interface))
        if mode != "disabled" and channel is None:
            raise Exception("Could not read interface %s channel"%(self.interface))

        # enabled
        if mode == "disabled":
            self.current_config['enabled'] = False
        else:
            self.current_config['enabled'] = True

        # mode
        if mode in ["a-only", "b-only", "g-only"]:
            self.current_config['mode'] = mode[0]
            self.current_config['ieee80211n'] = False
        elif mode == "bg-mixed":
            self.current_config['mode'] = "g"
            self.current_config['ieee80211n'] = False
        elif mode == "mixed" and self.band == IEEE80211_Channels.BAND_2400_MHz:
            self.current_config['mode'] = "g"
            self.current_config['ieee80211n'] = True
        elif mode == "mixed" and self.band == IEEE80211_Channels.BAND_5000_MHz:
            self.current_config['mode'] = "a"
            self.current_config['ieee80211n'] = True
        elif mode == "n-only":
            self.current_config['ieee80211n'] = True
        else:
            self.current_config['mode'] = "unknown"
            self.current_config['ieee80211n'] = False

        #ssid
        self.current_config['ssid'] = ssid

        #freq
        self.current_config['freq'] = float(IEEE80211_Channels.get_freq(channel, self.band))

        bitrate, txpower_mw, wmm = self.get_wireless_advanced_params()
        if mode != "disabled" and (bitrate is None or txpower_mw is None or wmm is None):
            raise Exception("Could not read bitrate or txpower_mw or wmm: %s, %s, %s"%
                            (str(bitrate), str(txpower_mw), str(wmm)))
        
        #bitrate
        self.current_config['bitrate'] = bitrate
        #txpower
        self.current_config['txpower_auto'] = False
        self.current_config['txpower'] = self.mw_to_dbm(txpower_mw)
        #wmm
        self.current_config['wmm'] = wmm

        enc_mode, enc_pass = self.get_wireless_security_params()
        if mode != "disabled" and (enc_mode is None or enc_pass is None):
            raise Exception("Could not read encryption mode")

        self.current_config['encryption_mode'] = enc_mode 
        self.current_config['encryption_pass'] = enc_pass

    def set_wireless_basic(self, ssid, net_mode, channel):
        req_args = {}
        req_args["change_action"] = "gozila_cgi"
        req_args["submit_button"] = "Wireless_Basic"
        req_args["submit_type"] = "save"
        
        req_args["%s_mode"%(self.interface)] = "ap"
        req_args["%s_net_mode"%(self.interface)] = net_mode
        if ssid is not None:
            req_args["%s_ssid"%(self.interface)] = ssid
        if channel is not None:
            req_args["%s_channel"%(self.interface)] =  channel
        
        req_args.update(self.get_other_interfaces_args())

        #self.apply_wrt_request(req_args, "Wireless_Basic") # "Save"
        req_args["action"] = "ApplyTake" 
        self.apply_wrt_request(req_args, "Wireless_Basic") # "Apply Settings"

    def set_wireless_advanced(self, bitrate, txpower, wmm):
        req_args = {}
        req_args["change_action"] = ""	
        req_args["commit"] = "1"
        req_args["interface"] = self.interface
        req_args["submit_button"] = "Wireless_Advanced-%s"%(self.interface)
        req_args["submit_type"] = "save"

        req_args["%s_rate"%self.interface] = bitrate
        req_args["%s_txpwr"%(self.interface)] = txpower
        if wmm == True:
            req_args["%s_wme"%(self.interface)] = "on"
        elif wmm == False:
            req_args["%s_wme"%(self.interface)] = "off"

        self.apply_wrt_request(req_args, "Wireless_Advanced-%s") # "Save"
        req_args["action"] = "ApplyTake"
        self.apply_wrt_request(req_args, "Wireless_Advanced-%s"%(self.interface))
        
    def set_wireless_security(self, encryption_mode, encryption_passkey):
        req_args = {}
        req_args["action"] = "ApplyTake"
        req_args["change_action"] = "gozila_cgi"
        req_args["submit_button"] = "WL_WPATable"
        req_args["submit_type"] = "save"
#        req_args["wl%d_security_mode"%(not int(self.interface[-1]))] = "disabled"
        if encryption_mode == ApControlConfig.ApControl_open:
            req_args["%s_security_mode"%(self.interface)] = "disabled"
        elif encryption_mode == ApControlConfig.ApControl_wep:
            req_args["%s_security_mode"%(self.interface)] = "wep"
            req_args["%s_WEP_key"%(self.interface)] = ""
            req_args["%s_key"%(self.interface)]= "1"
            req_args["%s_key1"%(self.interface)] = encryption_passkey
            req_args["%s_wep"%(self.interface)] = "restricted"
            req_args["%s_wep_bit"%(self.interface)] = "64" # supported: 64 or 128 bits
        elif encryption_mode in [ApControlConfig.ApControl_wpa,
                                 ApControlConfig.ApControl_wpa2,
                                 ApControlConfig.ApControl_wpa_wpa2]:
            if encryption_mode == ApControlConfig.ApControl_wpa:
                req_args["%s_security_mode"%(self.interface)] = "psk"
            elif encryption_mode == ApControlConfig.ApControl_wpa2:
                req_args["%s_security_mode"%(self.interface)] = "psk2"
            elif encryption_mode == ApControlConfig.ApControl_wpa_wpa2:
                req_args["%s_security_mode"%(self.interface)] = "psk psk2"
            req_args["%s_crypto"%(self.interface)] = "tkip"
            req_args["%s_wpa_gtk_rekey"%(self.interface)] = "3600" 
            req_args["%s_wpa_psk"%(self.interface)] = encryption_passkey
        else:
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] += "encryption mode %s not supported"%(encryption_mode)
            return

        #self.apply_wrt_request(req_args, "WL_WPATable") # "Save"
        #req_args["action"] = "ApplyTake" 
        self.apply_wrt_request(req_args, "WL_WPATable") # "Apply Settings"

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

        if config['enabled'] and \
                ((config['mode'] == "a" and self.band != IEEE80211_Channels.BAND_5000_MHz) or \
                ((config['mode'] == "b" or config['mode'] == "g") and self.band != IEEE80211_Channels.BAND_2400_MHz)):
            config['enabled'] = False
            self.current_config['status'] = "FAIL"
            self.current_config['errmsg'] = "Cannot set 802.11%s mode for interface in %s"%(config['mode'], self.band)
            
        change = False
        # enabled 
        if not config['enabled']:
            if self.current_config['enabled']:
                self.set_wireless_basic(None, "disabled", None)
                change = True
        else:
            # bitrate, txpower, wmm
            if config['txpower'] != self.current_config['txpower'] or \
                    config['bitrate'] != self.current_config['bitrate'] or \
                    config['wmm'] != self.current_config['wmm']:

                new_txpower_mw = self.dbm_to_mw(config['txpower']) 
                if new_txpower_mw > self.max_txpower_mw:
                    config['txpower'] = self.mw_to_dbm(self.max_txpower_mw)
                if new_txpower_mw < self.min_txpower_mw:
                    config['txpower'] = self.mw_to_dbm(self.min_txpower_mw)

                self.set_wireless_advanced(config['bitrate'], self.dbm_to_mw(config['txpower']), config['wmm'])
                change = True
    
            # security params:
            if config['encryption_mode'] != self.current_config['encryption_mode'] or \
                    (config['encryption_mode'] != "open" and \
                         config['encryption_pass'] != self.current_config['encryption_pass']):
                self.set_wireless_security(config['encryption_mode'], config['encryption_pass'])
                change = True

            # ssid, freq, mode, ieee80211n
            if config['enabled'] != self.current_config['enabled'] or \
                    config['ssid'] != self.current_config['ssid'] or \
                    config['freq'] != self.current_config['freq'] or \
                    config['mode'] != self.current_config['mode'] or \
                    config['ieee80211n'] != self.current_config['ieee80211n']:
                new_channel = IEEE80211_Channels.get_channel(config['freq'])
                if not config['enabled']:
                    new_mode = "disabled"
                elif config['ieee80211n']:
                    new_mode = "mixed"
                else: 
                    new_mode = config['mode'] + "-only"
                    
                self.set_wireless_basic(config['ssid'], new_mode, new_channel)
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
    rospy.init_node("ddwrt_apcontrol_node")

    ip = rospy.get_param("~ip", "192.168.1.1") 
    user = rospy.get_param("~user", "root") 
    password = rospy.get_param("~password", "admin") 
    interface = rospy.get_param("~interface", "wl0")

    ap = DdwrtApControl(ip, user, password, interface)

    dynamic_reconfigure.server.Server(ApControlConfig, ap.reconfigure)

    rospy.spin()
