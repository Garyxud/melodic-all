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

Object representing a name server node in the tree.

'''


import CosNaming
from omniORB import CORBA, TRANSIENT_ConnectFailed

from rtctree.exceptions import *
from rtctree.directory import Directory


##############################################################################
## Name server node object

class NameServer(Directory):
    '''Node representing a name server.

    Name server nodes should only be present in the first level of the tree.
    They can contain directories, managers and components as children.

    This class is a specialisation of the Directory class. It adds the
    functionality necessary for connecting to a name server and getting the
    root context.

    '''
    def __init__(self, orb=None, address=None, parent=None, filter=[],
                 *args, **kwargs):
        '''Constructor.

        @param orb An orb object to use to connect to the name server.
        @param address The address of the name server. Used as the node name.
        @param parent The parent node of this node, if any.
        @param filter A list of paths to filter by.

        '''
        super(NameServer, self).__init__(name=address, parent=parent,
                filter=filter, *args, **kwargs)
        self._parse_server(address, orb, filter)

    @property
    def is_nameserver(self):
        '''Is this node a name server (specialisation of directory nodes)?'''
        return True

    @property
    def orb(self):
        '''The ORB used to access this name server.'''
        with self._mutex:
            return self._orb

    @property
    def ns_object(self):
        '''The object representing this name server.'''
        with self._mutex:
            return self._ns_obj

    def _parse_server(self, address, orb, filter=[]):
        # Parse the name server.
        with self._mutex:
            self._address = address
            self._orb = orb
            root_context = self._connect_to_naming_service(address)
            self._parse_context(root_context, orb, filter)

    def _connect_to_naming_service(self, address):
        # Try to connect to a name server and get the root naming context.
        with self._mutex:
            self._full_address = 'corbaloc::{0}/NameService'.format(address)
            try:
                self._ns_obj = self._orb.string_to_object(self._full_address)
            except CORBA.ORB.InvalidName:
                raise InvalidServiceError(address)
            try:
                root_context = self._ns_obj._narrow(CosNaming.NamingContext)
            except CORBA.TRANSIENT, e:
                if e.args[0] == TRANSIENT_ConnectFailed:
                    raise InvalidServiceError(address)
                else:
                    raise
            if CORBA.is_nil(root_context):
                raise FailedToNarrowRootNamingError(address)
            return root_context


# vim: tw=79

