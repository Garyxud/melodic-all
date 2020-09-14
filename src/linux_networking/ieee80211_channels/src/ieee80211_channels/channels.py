#! /usr/bin/env python

freq_to_chan_map = {
    2412000000L : 1, 
    2417000000L : 2, 
    2422000000L : 3, 
    2427000000L : 4, 
    2432000000L : 5, 
    2437000000L : 6, 
    2442000000L : 7, 
    2447000000L : 8, 
    2452000000L : 9, 
    2457000000L : 10, 
    2462000000L : 11, 
    2467000000L : 12, 
    2472000000L : 13, 
    2484000000L : 14, 
    3657500000L : 131, 
    3662500000L : 132, 
    3660000000L : 132, 
    3667500000L : 133, 
    3665000000L : 133, 
    3672500000L : 134, 
    3670000000L : 134, 
    3677500000L : 135, 
    3682500000L : 136, 
    3680000000L : 136, 
    3687500000L : 137, 
    3685000000L : 137, 
    3689500000L : 138, 
    3690000000L : 138, 
    4915000000L : 183, 
    4920000000L : 184, 
    4925000000L : 185, 
    4935000000L : 187, 
    4940000000L : 188, 
    4945000000L : 189, 
    4960000000L : 192, 
    4980000000L : 196, 
    5035000000L : 7, 
    5040000000L : 8, 
    5045000000L : 9, 
    5055000000L : 11, 
    5060000000L : 12, 
    5080000000L : 16, 
    5170000000L : 34, 
    5180000000L : 36, 
    5190000000L : 38, 
    5200000000L : 40, 
    5210000000L : 42, 
    5220000000L : 44, 
    5230000000L : 46, 
    5240000000L : 48, 
    5260000000L : 52, 
    5280000000L : 56, 
    5300000000L : 60, 
    5320000000L : 64, 
    5500000000L : 100, 
    5520000000L : 104, 
    5540000000L : 108, 
    5560000000L : 112, 
    5580000000L : 116, 
    5600000000L : 120, 
    5620000000L : 124, 
    5640000000L : 128, 
    5660000000L : 132, 
    5680000000L : 136, 
    5700000000L : 140, 
    5745000000L : 149, 
    5765000000L : 153, 
    5785000000L : 157, 
    5805000000L : 161, 
    5825000000L : 165, 
}

class IEEE80211_Channels:
    """
    This class implements IEEE802.11 frequency <---> (channel, band) mapping.
    """
    BAND_2400_MHz = "2400MHz band" #: 2.4GHz band
    BAND_3600_MHz = "3600MHz band" #: 3.6GHz band
    BAND_5000_MHz = "5000MHz band" #: 5GHz band
    BAND_UNKNOWN = "Unknown band"  #: unknown band

    @staticmethod
    def get_band_from_freq(freq):
        """
        Retrieves the band a frequency belongs to.

        @type freq: long
        @param freq: the frequency in Hz

        @rtype: string
        @return: the frequency band (see the class constants)
        """
        if freq >= 2412000000L and freq <= 2484000000L: 
            return IEEE80211_Channels.BAND_2400_MHz
        elif freq >= 3657500000L and freq <= 3690000000L:
            return IEEE80211_Channels.BAND_3600_MHz
        elif freq >= 4915000000L and freq <= 5825000000L:
            return IEEE80211_Channels.BAND_5000_MHz
        else:
            return IEEE80211_Channels.BAND_UNKNOWN

    @staticmethod
    def get_channel(freq):
        """
        Returns the channel number corresponding to a frequency.
        
        @type freq: long
        @param freq: the frequency in Hz

        @rtype: int
        @return: channel number or -1 if the frequency is not a valid IEEE802.11 channel
        """
        try:
            return freq_to_chan_map[freq]
        except:
            return -1

    @staticmethod
    def get_freq(channel, band):
        """
        Returns the frequency corresponding to a given channel and band.
        
        @type channel: int
        @param channel: the channel number
        @type band: string
        @param band: the frequency band (one of the class defined constants should be given)

        @rtype: long
        @return: frequency in Hz or -1 if the (channel, band) combination is not valid
        """
        for freq, ch in freq_to_chan_map.iteritems():
            if ch == channel and IEEE80211_Channels.get_band_from_freq(freq) == band:
                return freq
        return -1
