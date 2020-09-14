from __future__ import with_statement
import radio
from netlink_monitor import netlink_monitor, IFSTATE
import async_helpers
import asmach as smach
from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks, returnValue
import state_publisher
import operator
import event

class RadioSmData:
    def __init__(self, iface_name):
        self._radio = radio.Radio(iface_name)
        self.iface_name = iface_name
        self._up_state_pub = netlink_monitor.get_state_publisher(iface_name, IFSTATE.UP)
        self.activate_request = state_publisher.StatePublisher(False)
        self.is_active = state_publisher.StatePublisher(False)
        self.scanning_enabled = state_publisher.StatePublisher(False)
        self.associate_request = event.Event()
        self.disactivate_delay = 1
        self.pre_up_sleep = 1
        
        # Copy radio members that we are willing to expose
        self.unassociate = self._radio.unassociate
        self.associated = self._radio.associated
        self.scanning = self._radio.scanning
        self.scan = self._radio.scan
        self.scan_results_event = self._radio.scan_results_event
        self.frequency_list = self._radio.frequency_list

class RadioSmState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, input_keys=['radio'], output_keys=['radio'], *args, **kwargs)

class Down(RadioSmState):
    def __init__(self):
        RadioSmState.__init__(self, outcomes=['up'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        # Wait a bit here because buggy drivers don't like to do things
        # just as they are coming down.
        yield async_helpers.async_sleep(ud.radio.pre_up_sleep)
        yield async_helpers.wait_for_state(ud.radio._up_state_pub, operator.truth)
        returnValue('up')

class Unassociated(RadioSmState):
    def __init__(self):
        RadioSmState.__init__(self, outcomes=['associate', 'down'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        # If we got here then were forcibly disactivated, and the previous
        # activation request is moot.
        ud.radio.activate_request.set(False)

        # We may have landed here after the interface went down causing
        # disassociation. Quickly check for that before getting to work.
        if not ud.radio._up_state_pub.get():
            returnValue('down')
        
        # Use callLater so that no association request will arrive before
        # we select.
        reactor.callLater(0, ud.radio.scanning_enabled.set, True)
        assoc_event_stream = async_helpers.EventStream(ud.radio.associate_request)
        events = yield async_helpers.select(
                async_helpers.StateCondition(ud.radio._up_state_pub, operator.__not__ ),
                assoc_event_stream
                )
        ud.radio.scanning_enabled.set(False)

        if 1 in events:
            ud.radio.requested_bss = assoc_event_stream.get()
            returnValue('associate')
        returnValue('down')

class Associating(RadioSmState):
    def __init__(self):
        RadioSmState.__init__(self, outcomes=['associated', 'failed'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        ud.radio._radio.associate(*ud.radio.requested_bss[0], **ud.radio.requested_bss[1])
        
        #print "Associating before", ud.radio.associated.get()
        events = yield async_helpers.select(
                async_helpers.StateCondition(ud.radio._up_state_pub, operator.__not__ ),
                async_helpers.StateCondition(ud.radio._radio.associated, lambda x: x != radio.Associating))

        if 0 in events or ud.radio.associated.get() == radio.Unassociated:
            returnValue('failed')
        else:
            #print "Associated", ud.radio.iface_name, ud.radio.associated.get()
            returnValue('associated')

class Associated(RadioSmState):
    def __init__(self):
        RadioSmState.__init__(self, outcomes=['activate', 'reassociate', 'unassociated'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        # Use callLater so that no association request will arrive before
        # we select.
        reactor.callLater(0, ud.radio.scanning_enabled.set, True)
        assoc_event_stream = async_helpers.EventStream(ud.radio.associate_request)
        #print "Associated before", ud.radio.associated.get()
        events = yield async_helpers.select(
                async_helpers.StateCondition(ud.radio._up_state_pub, operator.__not__ ),
                async_helpers.StateCondition(ud.radio.activate_request, operator.truth),
                assoc_event_stream,
                )
        ud.radio.scanning_enabled.set(False)

        if 1 in events:
            returnValue('activate')
        if 2 in events:
            ud.radio.requested_bss = assoc_event_stream.get()
            #print "Assoc changed", ud.radio.iface_name, ud.radio.requested_bss
            returnValue('reassociate')
        returnValue('unassociated')

class Active(RadioSmState):
    def __init__(self):
        RadioSmState.__init__(self, outcomes=['disactivate', 'unassociated'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        events = yield async_helpers.select(
          async_helpers.StateCondition(ud.radio.activate_request, operator.__not__),
          async_helpers.StateCondition(ud.radio.associated, operator.__not__))
        if 0 in events:
            returnValue('disactivate')
        returnValue('unassociated')

class Disactivating(RadioSmState):
    def __init__(self):
        RadioSmState.__init__(self, outcomes=['done', 'activate', 'unassociated'])

    @inlineCallbacks
    def execute_async(self, ud):
        events = yield async_helpers.select(
          async_helpers.Timeout(ud.radio.disactivate_delay),
          async_helpers.StateCondition(ud.radio.activate_request, operator.truth),
          async_helpers.StateCondition(ud.radio.associated, operator.__not__))
        if 1 in events:
            returnValue('activate')
        if 2 in events:
            returnValue('unassociated')
        returnValue('done')

def _state_change_cb(ud, states):
    ud.radio.is_active.set("ACTIVE" in states)

def radio_sm(iface):
    sm = smach.StateMachine(outcomes=[], input_keys=['radio'])
    smadd = smach.StateMachine.add
    with sm:
        smadd('DOWN', Down(),  transitions = { 'up' : 'UNASSOCIATED' })
        smadd('UNASSOCIATED', Unassociated(), transitions = { 'associate' : 'ASSOCIATING', 'down' : 'DOWN'})
        smadd('ASSOCIATING', Associating(), transitions = { 'associated' : 'ASSOCIATED', 'failed' : 'DOWN'})
        smadd('ASSOCIATED', Associated(), transitions = { 'reassociate' : 'ASSOCIATING', 'activate' : 'ACTIVE', 'unassociated' : 'DOWN'})
        smadd('ACTIVE', Active(), transitions = { 'disactivate' : 'DISACTIVATING', 'unassociated' : 'DOWN'} )
        smadd('DISACTIVATING', Disactivating(), transitions = { 'done' : 'ASSOCIATED', 'activate' : 'ACTIVE', 'unassociated' : 'DOWN'} )
    ud = smach.UserData()
    ud.radio = RadioSmData(iface)
    sm.register_transition_cb(_state_change_cb)
    sm.execute_async(ud)
    return ud.radio
