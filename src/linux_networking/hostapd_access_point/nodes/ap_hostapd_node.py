#! /usr/bin/env python

import time
import subprocess
from array import array
import os, sys, signal
import socket, fcntl
import struct
import math

import roslib; roslib.load_manifest('hostapd_access_point')
import rospy
import dynamic_reconfigure.server
from access_point_control.cfg import ApControlConfig
from ieee80211_channels.channels import IEEE80211_Channels

# range for valid txpower search
MIN_TXPOWER = 0                 # dBm
MAX_TXPOWER = 30                # dBm

IFNAMSIZ = 16                   # interface name size
MAX_SSID_LENGTH = 32

# ioctl calls from wireless.h
SIOCGIWESSID = 0x8B1B           # get ESSID
SIOCSIWRATE = 0x8B20            # set default bit rate (bps)
SIOCGIWFREQ = 0x8B05            # get channel/frequency (Hz)
SIOCSIWTXPOW = 0x8B26           # set transmit power (dBm)
SIOCGIWTXPOW = 0x8B27           # get transmit power (dBm)

class ApHostapd:
    def __init__(self, interface, ip = None, netmask = None, hostapd = "hostapd"):
        self.hostapd = hostapd
        self.conffile = "/var/run/hostapd_" + interface + ".conf"
        self.pidfile = "/var/run/hostapd_" + interface + ".pid"
        self.dumpfile = "/var/run/hostapd_" + interface + ".dump"
        self.pid = None

        self.interface = interface
        self.interface_data = array('c', interface + '\0' * (IFNAMSIZ-len(interface)));
        self.ioctl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        subprocess.call(["ifconfig", self.interface, "up"])
        if ip is not None:
            if netmask is not None:
                subprocess.call(["ifconfig", self.interface, ip, "netmask", netmask])
            else:
                subprocess.call(["ifconfig", self.interface, ip])

        self.config = dict()
        self.paused = True

    def isrunning(self):
        if self.pid is None:
            try:
                self.pid = self.get_hostapd_pid()
            except IOError as err:
                return False
        isrunning = os.path.exists("/proc/%d"%(self.pid))
        if not isrunning:
            self.pid = None
        return isrunning

    def get_wpa_flag(self):
        if self.config['encryption_mode'] == ApControlConfig.ApControl_wpa:
            return 1
        if self.config['encryption_mode'] == ApControlConfig.ApControl_wpa2:
            return 2
        return 3

    def generate_conf(self):
        # create hostapd.conf
        f = open(self.conffile, 'w')

        channel = IEEE80211_Channels.get_channel(self.config['freq'])

        if channel < 0:
            raise ValueError(-1, "Frequency not a valid IEEE 802.11 channel")

        if (self.config['mode'] == 'a' and \
                IEEE80211_Channels.get_band_from_freq(self.config['freq']) != IEEE80211_Channels.BAND_5000_MHz) or \
            ((self.config['mode'] == 'b' or self.config['mode'] =='g') and \
                 IEEE80211_Channels.get_band_from_freq(self.config['freq']) != IEEE80211_Channels.BAND_2400_MHz):
            raise ValueError("Requested frequency is not valid for 802.11" + self.config['mode'] + " mode")

        f.write("""
driver=nl80211

logger_syslog=-1
logger_syslog_level=2
logger_stdout=-1
logger_stdout_level=0

ctrl_interface=/var/run/hostapd

ignore_broadcast_ssid=0

own_ip_addr=127.0.0.1
country_code=""" + self.config['country_code'] + """

interface=""" + self.interface + """
ssid=""" + self.config['ssid'] + """
hw_mode=""" + self.config['mode'] + """
channel=""" + str(channel) + """
ieee80211n=""" + str(int(self.config['ieee80211n'])) + """

dump_file=""" + self.dumpfile)

        if self.config['encryption_mode'] == ApControlConfig.ApControl_open:
            f.write("""
auth_algs=1""")
        elif self.config['encryption_mode'] == ApControlConfig.ApControl_wep:
            f.write("""
auth_algs=1
wep_default_key=0
wep_key0=""" + self.config['encryption_pass'])
        elif self.config['encryption_mode'] in \
                [ ApControlConfig.ApControl_wpa, 
                  ApControlConfig.ApControl_wpa2, 
                  ApControlConfig.ApControl_wpa_wpa2]:
            wpa_flag = self.get_wpa_flag()
            f.write("""
auth_algs=3
wpa=""" + str(wpa_flag) + """
wpa_passphrase=""" + self.config['encryption_pass'] + """
wpa_pairwise=CCMP TKIP""")

        if self.config['wmm']:
            f.write("""

wmm_enabled=1

wmm_ac_bk_cwmax=10
wmm_ac_bk_aifs=7
wmm_ac_bk_txop_limit=0
wmm_ac_bk_acm=0
wmm_ac_be_aifs=3
wmm_ac_be_txop_limit=0
wmm_ac_be_acm=0
wmm_ac_vi_aifs=2
wmm_ac_vi_acm=0
wmm_ac_vo_aifs=2
wmm_ac_vo_acm=0
""")
            if self.config['mode'] != "b":
                f.write("""
wmm_ac_bk_cwmin=4
wmm_ac_be_cwmin=4
wmm_ac_be_cwmax=10
wmm_ac_vi_cwmin=3
wmm_ac_vi_cwmax=4
wmm_ac_vi_txop_limit=94
wmm_ac_vo_cwmin=2
wmm_ac_vo_cwmax=3
wmm_ac_vo_txop_limit=47
""")
            else:
                f.write("""
wmm_ac_bk_cwmin=5
wmm_ac_be_cwmin=5
wmm_ac_be_cwmax=7
wmm_ac_vi_cwmin=4
wmm_ac_vi_cwmax=5
wmm_ac_vi_txop_limit=188
wmm_ac_vo_cwmin=3
wmm_ac_vo_cwmax=4
wmm_ac_vo_txop_limit=102
""")
        f.write("\n")
        f.close()

    def restart_hostapd(self):  
        self.stop_ap()
        self.start_ap()

    def get_ioctl_param(self, ioctl_request):
        iwreq_data = array('c')
        iwreq_data.extend(self.interface_data)
        iwreq_data.extend(struct.pack("iBBH", 0, 0, 0, 0))
        iwreq_data.extend('\0' * (32 - len(iwreq_data)))

        fcntl.ioctl(self.ioctl_sock.fileno(), ioctl_request, iwreq_data)
        return struct.unpack("iBBH", iwreq_data[16:24])

    def start_ap(self):
        self.generate_conf() 
        if self.isrunning():
            raise Exception("hostapd already running")
        
        p = subprocess.Popen([self.hostapd, self.conffile, "-B", "-P", self.pidfile], stdout=subprocess.PIPE, stderr=subprocess.PIPE) 
        out, err = p.communicate() 
        ret = p.wait()
        if ret != 0:
            print err, out
            raise Exception(ret, out)

        self.paused = False        

    def get_hostapd_pid(self):
        f = open(self.pidfile, 'r')
        pid = int(f.read())
        f.close()
        return pid

    def stop_ap(self):
        if self.isrunning():
            os.kill(self.pid, signal.SIGTERM)
        else:
            raise Exception("hostapd is dead")

        signal_time = time.time()
        while self.isrunning() and time.time() - signal_time < 3.0:
            time.sleep(0.1)
            
        if self.isrunning():
            rospy.logwarn("hostapd pid %d was not terminated by SIGTERM signal, wating some more...", 
                          self.pid)
            signal_time = time.time()
            while self.isrunning() and time.time() - signal_time < 7.0:            
                time.sleep(0.1)
            if self.isrunning():
                os.kill(self.pid, signal.SIGKILL)
                rospy.logerr("hostapd pid %d terminated by SIGKILL signal", self.pid)                
                raise Exception("could not kill hostapd")

        self.paused = True

    def get_ssid(self):
        ssid_buf = array('c', '\0' * MAX_SSID_LENGTH)
        ssid_buf_ptr, ssid_buf_len = ssid_buf.buffer_info()
        buf_ref = struct.pack('Pi', ssid_buf_ptr, ssid_buf_len)
        
        iwreq_data = array('c')
        iwreq_data.extend(self.interface_data)
        iwreq_data.extend(buf_ref)
        iwreq_data.extend('\0' * (32 - len(iwreq_data)))

        fcntl.ioctl(self.ioctl_sock.fileno(), SIOCGIWESSID, iwreq_data)
 
        return ssid_buf.tostring()
            
    def set_txpower(self, txpower, fixed = 1):
        iwreq_data = array('c')
        iwreq_data.extend(self.interface_data)
        iwreq_data.extend(struct.pack("iBBH", txpower, fixed, 0, 0))
        iwreq_data.extend('\0' * (32 - len(iwreq_data)))

        fcntl.ioctl(self.ioctl_sock.fileno(), SIOCSIWTXPOW, iwreq_data)

    def get_txpower(self):
        txpower, fixed, disable, flags = self.get_ioctl_param(SIOCGIWTXPOW)
        return txpower

    def get_freq(self):
        iwreq_data = array('c')
        iwreq_data.extend(self.interface_data)
        iwreq_data.extend('\0' * (32 - len(iwreq_data)))
        
        fcntl.ioctl(self.ioctl_sock.fileno(), SIOCGIWFREQ, iwreq_data)
        m, e = struct.unpack("ih", iwreq_data[16:22])        
        return float(m) * math.pow(10, e)

    def set_bitrate(self, bitrate, fixed = 1):
        iwreq_data = array('c')
        iwreq_data.extend(self.interface_data)
        iwreq_data.extend(struct.pack("iBBH", bitrate, fixed, 0, 0))
        iwreq_data.extend('\0' * (32 - len(iwreq_data)))                
        status = fcntl.ioctl(self.ioctl_sock.fileno(), SIOCSIWRATE, iwreq_data)

    def set_encryption(self, encryption_mode, encryption_pass):
        self.config['encryption_mode'] = encryption_mode
        self.config['encryption_pass'] = encryption_pass
        self.restart_hostapd() 
    
    def reconfigure(self, config, level):
        # save bitrate setting
        if 'bitrate' in self.config:
            old_bitrate = self.config['bitrate']

        # update with new parameters
        self.config.update(config)                

        # if any hostapd parameter changed, stop hostapd
        if 2 & level and not self.paused:
            self.stop_ap()             

        self.config['status'] = 'OK'
        self.config['errmsg'] = ''

        if config['enabled'] and self.paused:
            try:
                self.start_ap()      
            except Exception as e:
                self.config['enabled'] = False
                self.config['status'] = 'FAIL'
                self.config['errmsg'] = str(e)

        if not config['enabled'] and not self.paused:
            self.stop_ap()

        # tx-power
        if 4 & level:
            if config['txpower_auto']:
                self.set_txpower(-1, 0)
                self.config['txpower'] = self.get_txpower()
            else:
                txpower = config['txpower']
                txpower_is_set = False
                step = 0
                while not txpower_is_set and (txpower + step >= MIN_TXPOWER \
                        or txpower + step <= MAX_TXPOWER):
                    try:
                        if txpower + step >= MIN_TXPOWER and txpower + step <= MAX_TXPOWER:
                            self.set_txpower(txpower + step)
                        else:
                            raise Exception()
                        txpower_is_set = True
                    except:
                        step = -step
                        if step >= 0:
                            step = step + 1
                if txpower_is_set:
                    self.config['txpower'] = txpower + step
                else:
                    self.config['status'] = 'FAIL'
                    self.config['errmsg'] = self.config['errmsg'] + " Could not set a valid txpower within the search range"
                    self.config['txpower'] = self.get_txpower()

        # bitrate
        if 8 & level:
            try:
                if config['bitrate'] > 0:                
                    self.set_bitrate(config['bitrate'])                    
                else:
                    self.set_bitrate(-1, 0)
            except Exception as err:
                self.config['status'] = 'FAIL'
                self.config['errmsg'] = self.config['errmsg'] + "\nCould not set bitrate\n" + str(err)
                self.config['bitrate'] = old_bitrate

        return self.config 
        

if __name__ == "__main__":
    rospy.init_node("ap_hostapd_node")

    interface = rospy.get_param("~interface")
    args = dict()
    try:
        args["ip"] = rospy.get_param("~ip")
    except KeyError:
        pass
    try:
        args["netmask"] = rospy.get_param("~netmask")
    except KeyError:
        pass
    try:
        args["hostapd"] = rospy.get_param("~hostapd_path")
    except KeyError:
        pass

    ap = ApHostapd(interface, **args)
    if ap.isrunning():
        ap.stop_ap()

    try:
        dynamic_reconfigure.server.Server(ApControlConfig, ap.reconfigure)
        rospy.spin()
    finally:
        if ap.isrunning():
            ap.stop_ap()
