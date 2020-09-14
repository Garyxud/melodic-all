#!/usr/bin/env python

import sys, os
from xml.dom.minidom import parse, parseString
import xml.dom

reload(sys)
sys.setdefaultencoding('utf-8')

from parseColladaBase import replaceLibraryNode

import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='replace library nodes')
    parser.add_argument('filename', nargs=1)
    parser.add_argument('-O', '--output', help='output filename')
    args = parser.parse_args()

    obj = replaceLibraryNode()

    if obj.init(args.filename[0]):
        if args.output:
            f = open(args.output, 'wb')
            obj.writeDocument(f)
            f.close()
        else:
            obj.writeDocument(sys.stdout)
