#! /usr/bin/env python

import re

def same(s1, s2):
    return to_packed(s1) == to_packed(s2)

def is_packed(pk):
    return type(pk) == str and len(pk) == 6

def to_packed(s):
    if is_packed(s):
        return s
    return str_to_packed(s)

def to_str(s):
    if is_str(s):
        return s
    return packed_to_str(s)

def packed_to_str(pk):
    if is_packed(pk):
        return ":".join(("%02x"%ord(pk[i]) for i in range(6)))
    raise ValueError("A packed MAC address is a 6 character string.")

str_re_str = '^%s$'%(':'.join(6*[2*'[0-9A-Fa-f]']))
str_re = re.compile(str_re_str)

def is_str(str):
    return str_re.search(str) is not None

def str_to_packed(str):
    if is_str(str):
        return "".join(chr(int(h,16)) for h in str.split(':'))
    raise ValueError("A MAC address string is 6 two digit hex numbers separated by colons.")

def make_special():
    out = {}
    base_macs = {
            '00:24:6C:81:C2:2' : "01 Bike Rack",  
            '00:24:6C:82:3C:9' : "02 Research Mid",  
            '00:24:6C:82:48:1' : "03 Research Corner",  
            '00:24:6C:82:3A:0' : "04 Pool Room",  
            '00:24:6C:82:2F:3' : "05 Sanford Office",  
            '00:24:6C:81:B9:F' : "06 Boondocks",  
            '00:24:6C:81:BF:8' : "07 Kevin Corner",  
            '00:24:6C:82:54:B' : "08 Front Lobby",  
            '00:24:6C:82:45:E' : "09 Cafe",  
            '00:24:6C:82:24:8' : "10 Steve Office",  
            '00:24:6C:81:D5:E' : "11 Green Room",  
            '00:24:6C:82:4E:F' : "12 Server Room",  
        }
    variants = {
            "0" : "B",
            "1" : "B-WEP",
            "2" : "B-WPA2",
            "8" : "A",
            "9" : "A-WEP",
            "A" : "A-WPA2",
            }
    for mac, descr in base_macs.iteritems():
        for var, vdescr in variants.iteritems():
            out[mac+var] = descr + " " + vdescr
    return out

special_macs = make_special()

def pretty(str):
    if is_str(str):
        mac = str
    elif is_packed(str):
        mac = packed_to_str(str)
    else:
        mac = None

    if mac:
        mac = mac.upper()
        if mac in special_macs:
            out = special_macs[mac]
        else:
            out = mac
    else:
        out = "Invalid MAC"
    
    return out

if __name__ == "__main__":
    import unittest
    import sys
        
    class BasicTest(unittest.TestCase):
        def test_bad_packed(self):
            self.assertRaises(ValueError, packed_to_str, "12345")
            self.assertRaises(ValueError, packed_to_str, "1234567")
            self.assertRaises(ValueError, packed_to_str, 12)
            self.assertRaises(ValueError, packed_to_str, [])

        def test_bad_str(self):
            self.assertRaises(ValueError, str_to_packed, "12:34:56:78:90")
            self.assertRaises(ValueError, str_to_packed, "12:34:56:78:90:ab:cd")
            self.assertRaises(ValueError, str_to_packed, "aoeu")
            self.assertRaises(ValueError, str_to_packed, "gg:12:34:56:78:90")

        def test_conv(self):
            self.assertEqual("blaise", str_to_packed("62:6c:61:69:73:65"))
            self.assertEqual("blaise", str_to_packed("62:6C:61:69:73:65"))
            self.assertEqual("blaise", str_to_packed(packed_to_str("blaise")))
            self.assertEqual("ct,cu ", str_to_packed(packed_to_str("ct,cu ")))

    if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
        import roslib; roslib.load_manifest('multi_interface_roam')
        import rostest
        rostest.unitrun('multi_interface_roam', 'addr_basic', BasicTest)
    else:
        unittest.main()
