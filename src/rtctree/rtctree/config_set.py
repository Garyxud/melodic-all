# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtctree

Copyright (C) 2009-2014
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

Object representing a configuration set.

'''


from rtctree.utils import dict_to_nvlist

##############################################################################
## Configuration set object

class ConfigurationSet(object):
    '''A class representing a configuration set.'''
    def __init__(self, owner=None, object=None, description=None, data=None,
                 *args, **kwargs):
        '''Constructor.

        @param owner The owner of this configuration set, if any. Should be a
                     Component object or None.
        @param object The CORBA ConfigurationSet object to wrap.
        @param description A description of this configuration set.
        @param data The dictionary containing the parameters and their values
                    of this configuration set.

        '''
        super(ConfigurationSet, self).__init__(*args, **kwargs)
        self._owner = owner
        self._object = object
        self._description = description
        self._data = data

    def has_param(self, param):
        '''Check if this configuration set has the given parameter.'''
        return param in self.data

    def set_param(self, param, value):
        '''Set a parameter in this configuration set.'''
        self.data[param] = value
        self._object.configuration_data = dict_to_nvlist(self.data)

    @property
    def data(self):
        '''Read-only access to the configuration set's parameters.'''
        return self._data

    @property
    def description(self):
        '''Read-only access to the configuration set's description.'''
        return self._description

    @property
    def object(self):
        '''The CORBA ConfigurationSet object this object wraps.'''
        return self._object

    def _reload(self, object, description, data):
        '''Reload the configuration set data.'''
        self._object = object
        self._description = description
        self._data = data


# vim: tw=79

