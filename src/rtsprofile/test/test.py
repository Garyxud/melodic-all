# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtsprofile

Copyright (C) 2009-2010
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

File: test.py

Unit tests.

'''

__version__ = '$Revision: $'
# $Source$


import os.path
from rtsprofile.rts_profile import RtsProfile
import sys
import tempfile
from traceback import print_exc


def main(argv):
    print \
'''This test loads a given XML or YAML file (type determined by the extension)
into an RtsProfile object. The object is printed to a string, which is stored.

It then attempts to save the object using the XML output. This output is loaded
back in, printed to a string, and that string compared to the original. They
should be the same.

This save-load-check process is then repeated for the YAML output.
'''

    # Load the input
    input_name = argv[1]
    type = os.path.splitext(input_name)[1][1:]
    f = open(input_name)
    if type == 'xml':
        orig_prof = RtsProfile(xml_spec=f)
    elif type == 'yaml':
        orig_prof = RtsProfile(yaml_spec=f)
    else:
        print >>sys.stderr, 'Unknown input type: {0}'.format(type)
        return 1
    f.close()
    orig_prof_str = str(orig_prof)
    print 'Loaded original.'
    print

    # Test XML output
    failed = False
    xml_output = ''
    xml_prof_str = ''
    try:
        xml_output = orig_prof.save_to_xml()
        print 'Saved as XML.'
        xml_prof = RtsProfile(xml_spec=xml_output)
        print 'Loaded XML.'
        xml_prof_str = str(xml_prof)
        print 'Printed XML.'
        print
    except:
        print_exc()
        print
        failed = True
    if xml_prof_str != orig_prof_str:
        print 'XML profile does not equal original profile.'
        failed = True
    if failed:
        print >>sys.stderr, 'XML test failed.'
        f = open('original_prof.dump', 'w')
        f.write(orig_prof_str)
        f.close()
        f = open('xml_prof.dump', 'w')
        f.write(xml_prof_str)
        f.close()
        f = open('xml_raw.dump', 'w')
        f.write(xml_output)
        f.close()
        return 1

    # Test YAML output
    failed = False
    yaml_output = ''
    yaml_prof_str = ''
    try:
        yaml_output = orig_prof.save_to_yaml()
        print 'Saved as YAML.'
        yaml_prof = RtsProfile(yaml_spec=yaml_output)
        print 'Loaded YAML.'
        yaml_prof_str = str(yaml_prof)
        print 'Printed YAML.'
        print
    except:
        print_exc()
        print
        failed = True
    if yaml_prof_str != orig_prof_str:
        print 'YAML profile does not equal original profile.'
        failed = True
    if failed:
        print >>sys.stderr, 'YAML test failed.'
        f = open('original_prof.dump', 'w')
        f.write(orig_prof_str)
        f.close()
        f = open('yaml_prof.dump', 'w')
        f.write(yaml_prof_str)
        f.close()
        f = open('yaml_raw.dump', 'w')
        f.write(yaml_output)
        f.close()
        return 1

    print >>sys.stderr, 'Tests passed.'
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))


# vim: tw=79

