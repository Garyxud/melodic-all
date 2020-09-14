#! /usr/bin/env python

import roslib; roslib.load_manifest('ieee80211_channels')
from ieee80211_channels.channels import *

def get_band_string(freq):
    band = IEEE80211_Channels.get_band_from_freq(freq)
    if band == IEEE80211_Channels.BAND_2400_MHz:
        return "<rowstyle=\"background-color: #FFFFE0;\">2.4GHz"
    elif band == IEEE80211_Channels.BAND_5000_MHz:
        return "<rowstyle=\"background-color: #FFE0E0;\">5GHz"
    elif band == IEEE80211_Channels.BAND_3600_MHz:
        return "<rowstyle=\"background-color: #E0E0FF;\">3.6GHz"
    else:
        raise Exception("Invalid band")

print "||<tablestyle=\"text-align: center;\">'''Band'''||'''Frequency'''||'''Channel'''||"
freqs = freq_to_chan_map.keys()
freqs.sort()
for freq in freqs:
    print "||%s||%.1f MHz||%d||"%(get_band_string(freq), freq/1e6, freq_to_chan_map[freq])
