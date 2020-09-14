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

File: direction.py

Location object storing the position of components in a graphical view.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_EXT_NS, RTS_EXT_NS_S
from rtsprofile import direction as dir
from rtsprofile.utils import validate_attribute


##############################################################################
## Location object

class Location(object):
    '''Stores the location of a component in a graphical view.'''

    def __init__(self, x=0, y=0, height=0, width=0, direction=dir.DOWN):
        '''Constructor.

        @param x X position of the top-left of the component.
        @type x int
        @param y Y position of the top-left of the component.
        @type y int
        @param height Height of the component.
        @type height int
        @param width Width of the component.
        @type width int
        @param direction Direction the component faces.
        @type direction direction.const_type

        '''
        validate_attribute(x, 'Location.x',
                           expected_type=int, required=False)
        self._x = x
        validate_attribute(y, 'Location.y',
                           expected_type=int, required=False)
        self._y = y
        validate_attribute(height, 'Location.height',
                           expected_type=int, required=False)
        self._height = height
        validate_attribute(width, 'Location.width',
                           expected_type=int, required=False)
        self._width = width
        validate_attribute(direction, 'Location.direction',
                           expected_type=dir.const_type, required=False)
        self._direction = direction

    def __str__(self):
        return 'Position: {0}, {1}\nSize: {2}x{3}\nDirection: {4}'.format(\
                self.x, self.y, self.width, self.height,
                dir.to_string(self.direction))

    @property
    def x(self):
        '''The X position of the component in a graphical tool.'''
        return self._x

    @x.setter
    def x(self, x):
        validate_attribute(x, 'Location.x',
                           expected_type=int, required=False)
        self._x = x

    @property
    def y(self):
        '''The Y position of the component in a graphical tool.'''
        return self._y

    @y.setter
    def y(self, y):
        validate_attribute(y, 'Location.y',
                           expected_type=int, required=False)
        self._y = y

    @property
    def height(self):
        '''The height of the component in a graphical tool.'''
        return self._height

    @height.setter
    def height(self, height):
        validate_attribute(height, 'Location.height',
                           expected_type=int, required=False)
        self._height = height

    @property
    def width(self):
        '''The width of the component in a graphical tool.

        A value of -1 for this property indicates that the width should be as
        wide as is necessary.

        '''
        return self._width

    @width.setter
    def width(self, width):
        validate_attribute(width, 'Location.width',
                           expected_type=int, required=False)
        self._width = width

    @property
    def direction(self):
        '''The direction of the component in a graphical tool.

        A value of -1 for this property indicates that the height should be as
        wide as is necessary.

        '''
        return self._direction

    @direction.setter
    def direction(self, direction):
        validate_attribute(direction, 'Location.direction',
                           expected_type=dir.const_type, required=False)
        self._direction = direction

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a location into this
        object.

        '''
        self.x = int(node.getAttributeNS(RTS_EXT_NS, 'x'))
        self.y = int(node.getAttributeNS(RTS_EXT_NS, 'y'))
        self.height = int(node.getAttributeNS(RTS_EXT_NS, 'height'))
        self.width = int(node.getAttributeNS(RTS_EXT_NS, 'width'))
        self.direction = dir.from_string(node.getAttributeNS(RTS_EXT_NS,
                'direction'))
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a location into this object.'''
        self.x = int(y['x'])
        self.y = int(y['y'])
        self.height = int(y['height'])
        self.width = int(y['width'])
        self.direction = dir.from_string(y['direction'])
        return self

    def save_xml(self, doc, element):
        '''Save this location into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'x', str(self.x))
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'y', str(self.y))
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'height',
                               str(self.height))
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'width',
                               str(self.width))
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'direction',
                               dir.to_string(self.direction))

    def to_dict(self):
        '''Save this location into a dictionary.'''
        return {'x': self.x,
                'y': self.y,
                'height': self.height,
                'width': self.width,
                'direction': dir.to_string(self.direction)}


# vim: tw=79

