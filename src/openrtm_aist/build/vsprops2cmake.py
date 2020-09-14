#!/usr/bin/env python

from xml.dom import minidom, Node
import sys
import re
import os

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "please specify vsprops file"
        sys.exit(1)

    fname = sys.argv[1]
    if fname.split(".")[-1] != "vsprops":
        print "please specify vsprops file"
        sys.exit(1)

    f = file(sys.argv[1], "r")
    text = f.read().replace("shift_jis", "utf-8")
    f.close()

    doc = minidom.parseString(text)

    cmakefname = fname.replace(".vsprops", ".cmake")
    cf = file(cmakefname, "w")

    for node in doc.getElementsByTagName('UserMacro'):
        name = node.getAttribute("Name")
        value = node.getAttribute("Value")

        # escape backslash
        value = value.replace("\\", "\\\\")
        # escape quote
        value = re.sub("\"", "\\\"", value)
        # replace environment variable specifier
        value = re.sub("\%(.*?)\%", "$ENV{\\1}", value)
        # repalce variable expression (vsprops->cmake)
        value = re.sub("\$\((.*?)\)", "${\\1}", value)
        # keep solution directory variable
        value = re.sub("{SolutionDir}", "(SolutionDir)", value)

        cf.write("set (" + name + " \"" + value + "\")\n")

    cf.close()
