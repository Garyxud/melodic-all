#!/usr/bin/env python
#
# @brief Visual Studio Project file generator
# @date $Date: 2007-07-20 15:36:59 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2007
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

import sys
import getopt
import ezt
import uuid
import yaml

def print_usage():
    print """usage:
%s -i [input_file] -d [dictionary] -o [output_file]
"""


def load_dict(filename):
    f = open(filename, "r")
    return yaml.load(f.read())

def generate(ifile, ofile, dict):
    ifd = open(ifile, "r")
    itext = ifd.read()
    ifd.close()

    ofd = open(ofile, "w")
    t = ezt.Template(compress_whitespace = 0)
    t.parse(itext)
    t.generate(ofd, dict)
    ofd.close()

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "i:o:d:", [])
    except:
        print "Error: Invalid option.", getopt.GetoptError
        print_usage()
        sys.exit(-1)
        return

    if not opts:
        print_usage()
        sys.exit(-1)
        return

    for o, a in opts:
        if o in ("-i"):
            ifile = a
        if o in ("-o"):
            ofile = a
        if o in ("-d"):
            dfile = a
        if o in ("-h"):
            print_usage()
            sys.exit(0)
        # ...

    dict = load_dict(dfile)
    dict["ProjectGUID"] = str(uuid.uuid1())
    dict["SolutionGUID"] = str(uuid.uuid1())
    dict["SourceGUID"] = str(uuid.uuid1())
    dict["HeaderGUID"] = str(uuid.uuid1())
    dict["ResourceGUID"] = str(uuid.uuid1())
    generate(ifile, ofile, dict)

        


if __name__ == "__main__":
        main()
