from __future__ import with_statement
import roslib; roslib.load_manifest("multi_interface_roam")
import rospy
import wpa_supplicant_node.msg as wpa_msgs
import actionlib                       
import state_publisher
import mac_addr
import event
import threading
from twisted.internet.defer import Deferred
from twisted.internet import reactor
from async_helpers import mainThreadCallback

class Network:
    def __init__(self, id, enabled, parameters):
        self.id = id
        self.enabled = enabled
        if type(parameters) != dict:
            # assume it is a message
            parameters = dict((p.key, p.value) for p in parameters)
        self.parameters = parameters
        self.__getitem__ = parameters.__getitem__
        try:
            self.ssid = self['ssid'][1:-1]
        except KeyError: 
            self.ssid = None

class AssociationState(type):
    def __new__(meta, classname, bases, classDict):
        return type.__new__(meta, classname, bases, classDict)
    def __nonzero__(cls):
        return False

class Associating(object):
    __metaclass__ = AssociationState

class Unassociated(object):
    __metaclass__ = AssociationState

def is_not_associating(state):
    print state, id(state), id(Associating)
    return state != Associating 

class Radio:
    def __init__(self, interface_name):
        self.interface_name = interface_name
        self._last_assoc_msg = None
        self._associating = False
        self.associated = state_publisher.StatePublisher(Unassociated)
        self.frequency_list = state_publisher.StatePublisher([])
        self.network_list = state_publisher.StatePublisher([])
        self.scan_results_event = event.Event()
        self.scanning = state_publisher.StatePublisher(False)
        self.mutex = threading.Lock()

        #def debug_assoc(iface, old_state, new_state):
        #    print "Assoc change", iface, new_state
        #self.associated.subscribe(debug_assoc, interface_name)

        self.networks = []
        self.hidden_networks = []
        self.raw_scan_results = []
        self.scan_results = []
        self.scan_deferred = None

        prefix = rospy.resolve_name("wifi")+"/"+interface_name+"/"
        rospy.Subscriber(prefix + "scan_results", wpa_msgs.ScanResult, self._scan_results_callback)
        rospy.Subscriber(prefix + "network_list", wpa_msgs.NetworkList, self._network_list_callback)
        rospy.Subscriber(prefix + "frequency_list", wpa_msgs.FrequencyList, self._frequency_list_callback)
        self._scan_action = actionlib.SimpleActionClient(prefix+"scan", wpa_msgs.ScanAction)
        self._associate_action = actionlib.SimpleActionClient(prefix+"associate", wpa_msgs.AssociateAction)

    def is_scanning(self):
        return _scan_action.simple_state == SimpleGoalState.DONE
    
    def scan(self, freqs = [], ssids = []):  
        goal = wpa_msgs.ScanGoal()
        for s in ssids:
            goal.ssids.append(s)
        for f in freqs:
            goal.frequencies.append(f)
        #print "scan goal", goal
        self._scan_action.send_goal(goal, done_cb = self._scan_done_callback)
        self.scanning.set(True)
        self.scan_deferred = Deferred()
        return self.scan_deferred

    def cancel_scans(self):
        self.scanning = False
        _scan_action.cancel_all_goals()

    def associate(self, id):
        print "radio.associate", self.interface_name
        ssid = id[0]
        bssid = mac_addr.to_packed(id[1])
        with self.mutex:
            self._associate_action.send_goal(wpa_msgs.AssociateGoal(ssid, bssid), 
                    done_cb = self._assoc_done_cb, 
                    feedback_cb = self._assoc_feedback_cb)
        self.associated.set(Associating)

    def unassociate(self):
        self._associate_action.cancel_all_goals()

    @mainThreadCallback
    def _assoc_feedback_cb(self, fbk):
        print "_assoc_feedback_cb", self.interface_name, fbk.associated
        if fbk.associated:
            self.associated.set(fbk.bss)

    def _assoc_done_cb(self, state, rslt):
        # DANGER, this is not run in the main thread!!
        
        print "_assoc_done_cb", self.interface_name
        # Make sure that no feedback for this goal will arrive after we
        # have processed the goal.
        with self.mutex:
            self._associate_action.stop_tracking_goal()
        reactor.callFromThread(self.associated.set, Unassociated)
    
    @mainThreadCallback
    def _scan_done_callback(self, state, rslt):
        """Called when all our scans are done."""
        self.scanning.set(False)
        if self.scan_deferred:
            self.scan_deferred.callback(rslt.success)
        self.scan_deferred = None

    @mainThreadCallback
    def _frequency_list_callback(self, msg):
        self.frequency_list.set(msg.frequencies)

    @mainThreadCallback
    def _network_list_callback(self, msg):
        self.networks = [ Network(net.network_id, net.enabled, net.parameters) for net in msg.networks ]
        self.hidden_networks = [ net for net in self.networks if net.parameters['scan_ssid'] == '1' ]
        self.network_list.set(self.networks)
        self._filter_raw_scan_results() # The filtered scan results are out of date.

    def enabled_bss(self, bss):
        for net in self.networks:
            # @todo This check will not work for <any> netwoks. FIXME
            # @todo Might want to check that security parameters match too. FIXME
            if net.ssid == bss.ssid:
                return True
        return False

    def _filter_raw_scan_results(self):
        self.scan_results = [ bss for bss in self.raw_scan_results if self.enabled_bss(bss) ]
        self.scan_results = filter(lambda bss: bss.frequency in self.frequency_list.get(), self.scan_results)
        self.scan_results_event.trigger(self.scan_results)
    
    @mainThreadCallback
    def _scan_results_callback(self, msg):
        self.raw_scan_results = msg.bss
        self._filter_raw_scan_results() 

