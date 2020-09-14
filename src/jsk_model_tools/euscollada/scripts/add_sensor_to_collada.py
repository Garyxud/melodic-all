#!/usr/bin/env python

import sys, os
from xml.dom.minidom import parse, parseString
import xml.dom
import yaml
import argparse

reload(sys)
sys.setdefaultencoding('utf-8')

from parseColladaBase import parseXmlBase
from parseColladaBase import yamlParser

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='add_sensor_to_collada')
    parser.add_argument('filename', nargs=1)
    parser.add_argument('-O', '--output', help='output filename')
    parser.add_argument('-C', '--config', help='config filename (yaml file)')
    parser.add_argument('--without_sensor', default=False, action="store_true")
    parser.add_argument('--without_manipulator', default=False, action="store_true")
    parser.add_argument('--without_links', default=False, action="store_true")
    parser.add_argument('--without_replace_xmls', default=False, action="store_true")

    args = parser.parse_args()

    obj = parseXmlBase().readXmlFile(args.filename[0])

    if obj.init():
        if args.config:
            yaml_obj = yamlParser()
            yaml_obj.load(args.config)

            if not args.without_sensor:
                yaml_obj.add_sensor(obj)

            if not args.without_manipulator:
                yaml_obj.add_eef(obj)

            if not args.without_links:
                yaml_obj.add_links(obj)

            if not args.without_replace_xmls:
                yaml_obj.replace_xmls(obj)
        else:
            sys.stderr.write('no configuration file !\n')

        if args.output:
            f = open(args.output, 'wb')
            obj.writeDocument(f)
            f.close()
        else:
            obj.writeDocument(sys.stdout) ## wirting to standard output

## sample yaml for sensors
#sensors:
#  - {sensor_name: 'lhsensor',  sensor_type: 'force', parent_link: 'LARM_LINK6', translate: '0 0 0', rotate: '1 0 0 0'}
#  - {sensor_name: 'gsensor',   sensor_type: 'acceleration', parent_link: 'BODY'}
#  - {sensor_name: 'gyrometer', sensor_type: 'gyro', parent_link: 'BODY'}
#  - {sensor_name: 'camera', sensor_type: 'camera', parent_link: 'BODY'}
