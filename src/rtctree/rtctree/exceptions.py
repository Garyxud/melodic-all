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

General exception classes.

'''


import rtctree
import RTC


##############################################################################
## Exceptions

class RtcTreeError(Exception):
    '''Base error class.

    Used for undefined errors that are not core Python errors.

    '''
    pass


class ReturnCodeError(RtcTreeError):
    '''Generic error using a value of ReturnCode_t to set the message.'''
    def __init__(self, return_code):
        '''Constructor.

        @param return_code The type of return code. Must be on of the return
        codes defined in the RTC IDL.

        '''
        if return_code == RTC.RTC_ERROR:
            RtcTreeError.__init__(self, 'General error')
        elif return_code == RTC.BAD_PARAMETER:
            RtcTreeError.__init__(self, 'Bad parameter')
        elif return_code == RTC.UNSUPPORTED:
            RtcTreeError.__init__(self, 'Unsupported')
        elif return_code == RTC.OUT_OF_RESOURCES:
            RtcTreeError.__init__(self, 'Out of resources')
        elif return_code == RTC.PRECONDITION_NOT_MET:
            RtcTreeError.__init__(self, 'Precondition not met')


class InvalidServiceError(RtcTreeError):
    '''Could not connect to a CORBA service at an address.'''
    def __str__(self):
        return 'Invalid CORBA naming service: {0}'.format(self.args[0])


class FailedToNarrowRootNamingError(RtcTreeError):
    '''Failed to narrow the root naming context of a name server.'''
    def __str__(self):
        return 'Failed to narrow root naming context {0}'.format(self.args[0])


class NonRootPathError(RtcTreeError):
    '''A path did not begin with '/'.'''
    def __str__(self):
        return 'Path does not start at root: {0}'.format(self.args[0])


class CannotHoldChildrenError(RtcTreeError):
    '''Tried to add a child to a node that cannot hold children.'''
    def __str__(self):
        return 'Node cannot hold children.'


class BadECIndexError(RtcTreeError):
    '''Given the index of an execution context beyond the number of owned/
    participating contexts.

    '''
    def __str__(self):
        return 'Bad execution context index: {0}'.format(self.args[0])


class NoECWithHandleError(RtcTreeError):
    '''No execution context exists with the given handle.'''
    def __str__(self):
        return ('No execution context exists with ec_handle '
            '{0}'.format(self.args[0]))


class WrongPortTypeError(RtcTreeError):
    '''Tried to connect two ports of incompatible type.'''
    def __str__(self):
        return 'Wrong port type.'


class IncompatibleDataPortConnectionPropsError(RtcTreeError):
    '''Given incompatible properties for a connection between two data ports.'''
    def __str__(self):
        return 'Incompatible connection properties.'


class FailedToConnectError(ReturnCodeError):
    '''Failed to make a connection between two ports.'''
    def __str__(self):
        return 'Failed to make connection: {0}'.format(self.args[0])


class MismatchedInterfacesError(RtcTreeError):
    '''Interfaces between two service ports do not match type.'''
    def __str__(self):
        return 'Interfaces do not match.'


class MismatchedPolarityError(RtcTreeError):
    '''Interfaces between two service ports do not match polarity.'''
    def __str__(self):
        return 'Polarities do not match.'


class NotConnectedError(RtcTreeError):
    '''A connection is not connected.'''
    def __str__(self):
        return 'Not connected.'


class UnknownConnectionOwnerError(RtcTreeError):
    '''A connection's owning port is not known.'''
    def __str__(self):
        return 'Connection owner unknown.'


class NoSuchConfSetError(RtcTreeError):
    '''Attempted to access a configuration set that doesn't exist.'''
    def __str__(self):
        return 'No such configuration set: {0}'.format(self.args[0])


class NoSuchConfParamError(RtcTreeError):
    '''Attempted to access a configuration parameter that doesn't exist.'''
    def __str__(self):
        return 'No such configuration parameter: {0}'.format(self.args[0])


class NoSuchOptionError(RtcTreeError):
    '''The requested option has not been set.'''
    def __str__(self):
        return 'No such option: {0}'.format(self.args[0])


class BadPathError(RtcTreeError):
    '''Error indicating an invalid path.'''
    def __str__(self):
        return 'Bad path: {0}'.format(self.args[0])


class ManagerError(RtcTreeError):
    '''Base error type for errors involving managers.'''
    def __str__(self):
        return 'Unknown manager error'


class FailedToLoadModuleError(ManagerError):
    '''Error loading a shared library into a manager.'''
    def __str__(self):
        if len(self.args) == 1:
            return 'Failed to load module: {0}'.format(self.args[0])
        else:
            return 'Failed to load module: {0}'.format(self.args)


class FailedToUnloadModuleError(ManagerError):
    '''Error unloading a shared library from a manager.'''
    def __str__(self):
        return 'Failed to unload module: {0}'.format(self.args[0])


class FailedToCreateComponentError(ManagerError):
    '''Error creating a component out of a shared library in a manager.'''
    def __str__(self):
        return 'Failed to create component: {0}'.format(self.args[0])


class FailedToDeleteComponentError(ManagerError):
    '''Error deleting a component from a manager.'''
    def __str__(self):
        return 'Failed to delete component: {0}'.format(self.args[0])


class FailedToSetConfigurationError(ManagerError):
    '''Error setting a manager configuration parameter.'''
    def __str__(self):
        return 'Failed to set configuration: {0}'.format(self.args[0])


class FailedToAddMasterManagerError(ManagerError):
    '''Error when adding a master manager to another manager.'''
    def __str__(self):
        return 'Failed to add master manager.'


class FailedToRemoveMasterManagerError(ManagerError):
    '''Error when removing a master manager.'''
    def __str__(self):
        return 'Failed to remove master manager.'


class FailedToAddSlaveManagerError(ManagerError):
    '''Error when adding a slave manager to another manager.'''
    def __str__(self):
        return 'Failed to add slave manager: {0}'.format(self.args[1])


class FailedToRemoveSlaveManagerError(ManagerError):
    '''Error when removing a slave manager.'''
    def __str__(self):
        return 'Failed to remove slave manager: {0}'.format(self.args[1])


class NotRelatedError(RtcTreeError):
    '''Tried to manupulate the relationship between two nodes that are not
    parent and child.'''
    def __str__(self):
        return 'Nodes are not related: {0}, {1}'.format(self.args[0],
                self.args[1])


class NoSuchEventError(RtcTreeError):
    '''Tried to register a callback for a non-existent event.'''
    def __str__(self):
        return 'Callback event {0} does not exist on node {1}.'.format(
                self.args[1], self.args[0])


class AddLoggerError(RtcTreeError):
    '''Error adding a logger to a node.'''
    def __str__(self):
        return 'Error adding logger to node {0}.'.format(
                self.args[0])


class NoCBError(RtcTreeError):
    '''Tried to remove an unregistered callback.'''
    def __str__(self):
        return 'Callback {0}:{1} does not exist on node {2}.'.format(
                self.args[1], self.args[2], self.args[0])


class NoLoggerError(RtcTreeError):
    '''Tried to remove an unregistered logger.'''
    def __str__(self):
        return 'Logger {0} does not exist on node {1}.'.format(
                self.args[0], self.args[1])


class NotCompositeError(RtcTreeError):
    '''The component is not a composite component.'''
    def __str__(self):
        return 'Component {0} is not a composite component.'.format(
                self.args[0])


class NotInCompositionError(RtcTreeError):
    '''The component is not a member of the specified composite component.'''
    def __str__(self):
        return 'Component {0} is not a member of composition {1}.'.format(
                self.args[1], self.args[0])


class AlreadyInCompositionError(RtcTreeError):
    '''The component is already a member of the composite component.'''
    def __str__(self):
        return 'Component {0} is already a member of composition {1}.'.format(
                self.args[1], self.args[0])


class NotCORBANameError(RtcTreeError):
    '''A passed parameter is not a valid CORBA CosNaming.Name.'''
    def __str__(self):
        return 'Not a CORBA CosNaming.Name: {0}'.format(self.args[0])


# vim: tw=79

