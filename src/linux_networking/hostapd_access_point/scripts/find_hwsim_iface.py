#! /usr/bin/env python

import subprocess
import re
import sys

def get_hwsim_list():
    p = subprocess.Popen(["iwconfig"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (out, err) = p.communicate()

    iface_list = re.findall('(?m)^\S+', out)
    hwsim_list = []
    for iface in iface_list:
        f=open("/sys/class/net/" + iface + "/device/uevent")
        out = f.read()
        driver = re.findall('DRIVER.*(mac80211_hwsim)', out)
        if driver:
            hwsim_list.append(iface)

    return hwsim_list

def reload_hwsim_module(radios):
    ret = subprocess.call(["rmmod", "mac80211_hwsim"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    ret = subprocess.call(["modprobe", "mac80211_hwsim", "radios=" + str(radios)])
    if ret != 0:
        raise OSError(ret, "could not load mac80211_hwsim module")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: find_hwsim_iface.py <radio_index> <total_radios>"
        print "Example: find_hwsim_iface.py 1 2"
        sys.exit(1)
        
    curr_radio = int(sys.argv[1])
    total_radios = int(sys.argv[2])

    hwsim_list = get_hwsim_list()

    if len(hwsim_list) < total_radios:
        reload_hwsim_module(total_radios)
        
        hwsim_list = get_hwsim_list()
        
        if len(hwsim_list) < total_radios:
            raise OSError(-1, "could only spawn " + str(len(hwsim_list)) +
                           " radios instead of " + str(total_radios))
    
    if curr_radio < 1 or curr_radio > len(hwsim_list):
        raise IOError(-1, "radio index %d not available (total available radios = %d)"
                       %(curr_radio, total_radios))

    sys.stdout.write(hwsim_list[curr_radio - 1])


