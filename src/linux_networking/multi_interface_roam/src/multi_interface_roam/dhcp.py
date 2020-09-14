#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
import asmach as smach
import scapy.all as scapy
from twisted.internet.defer import inlineCallbacks, returnValue
from netlink_monitor import monitor, IFSTATE
import async_helpers
import l2socket
import event
import random
import time
import traceback
import ipaddr
import state_publisher
import config

# FIXME Add support for multiple leases.
# FIXME Add a way to signal x amount of time before we lose the lease.

class DhcpLease:
    def __init__(self):
        self.timeout_time = {}
        self.public_config = {}

class DhcpData:
    def __init__(self, iface):
        self.iface = iface
        self.socket = None
        self.link_addr_state = monitor.get_state_publisher(self.iface, IFSTATE.LINK_ADDR) 
        self.error_event = event.Event()
        self.binding_publisher = state_publisher.StatePublisher(None)
        self.error_timeout = 5
        self.exp_backoff_min = 0.2
        self.exp_backoff_max = 0.5
        self.exp_backoff_timeout = config.get_parameter('dhcp_timeout', 2)
        self.leases = {}

    def start_socket(self):
        if not self.socket or self.socket.port.fileno() == -1:
            self.socket = async_helpers.ReadDescrEventStream(l2socket.L2Port, iface = self.iface, 
                    filter='udp and dst port 68 and src port 67')

    def stop_socket(self):
        self.socket = None

class DhcpState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, input_keys=['dhcp'], output_keys=['dhcp'], *args, **kwargs)

def find_dhcp_option(key, default, dhcp):
    for opt in dhcp.options:
        if opt == key:
            return None
        if opt[0] == key:
            return opt[1:]
    return default


def dhcp_type_str(id):
    try:
        return scapy.DHCPTypes[id[0]]
    except KeyError:
        return "<unknown %s>"%repr(id)
 


class ExchangeRetryExponentialBackoff:
    def init_timeouts(self, ud):
        self.cur_retry_max = ud.dhcp.exp_backoff_min
        return ud.dhcp.exp_backoff_timeout

    def get_next_retry(self, ud):
        interval = self.cur_retry_max * random.uniform(0.5, 1)
        self.cur_retry_max = min(ud.dhcp.exp_backoff_max, 2 * self.cur_retry_max)
        return interval



class ExchangeRetryHalve:
    def init_timeouts(self, ud):
        return ud.dhcp.lease.timeout_time[self.type] - time.time()

    def get_next_retry(self, ud):
        return 60 + (ud.dhcp.lease.timeout_time[self.type] - time.time()) / 2


        
class ExchangeDiscover:
    message_type = "discover"

    def init_xid(self, ud):
        ud.dhcp.lease.xid = random.randint(0, 0xFFFF)

    def validate(self, ud, pkt):
        try:
            ip = pkt.payload
            udp = ip.payload 
            bootp = udp.payload
            dhcp = bootp.payload
            
            if not self.validate_common(ud, pkt, ip, udp, bootp, dhcp):
                print "validate_common returned False"
                return False

            message_type = dhcp_type_str(find_dhcp_option('message-type', None, dhcp))
            if message_type != 'offer':
                print "Ignoring packet based on unexpected message_type %s"%message_type
                return False
                
            ip = ud.dhcp.lease.public_config['ip'] = bootp.yiaddr
            ud.dhcp.lease.server_ip = find_dhcp_option('server_id', "0.0.0.0", dhcp)[0]
            ud.dhcp.lease.server_mac = pkt.src
            ud.dhcp.lease.public_config['gateway'] = find_dhcp_option('router', "0.0.0.0", dhcp)[0]
            netmask = ud.dhcp.lease.public_config['netmask'] = find_dhcp_option('subnet_mask', "0.0.0.0", dhcp)[0]
            net = ipaddr.IPv4Network("%s/%s"%(ip, netmask))
            ud.dhcp.lease.public_config['netmask_bits'] = net.prefixlen
            ud.dhcp.lease.public_config['network'] = net.network
            ud.dhcp.lease.public_config['ip_slashed'] = "%s/%i"%(ip, net.prefixlen)
            ud.dhcp.lease.public_config['network_slashed'] = "%s/%i"%(net.network, net.prefixlen)

            # TODO do more stuff here?
            return 'success'
            

        except:
            traceback.print_exc()
            print "Excepiton validating packet."
            return False


class ExchangeRequest:
    message_type = "request"
    
    def init_xid(self, ud):
        pass
        
    def validate(self, ud, pkt):
        try:
            ip = pkt.payload
            udp = ip.payload 
            bootp = udp.payload
            dhcp = bootp.payload
            
            if not self.validate_common(ud, pkt, ip, udp, bootp, dhcp):
                print "validate_common returned False"
                return False

            message_type = dhcp_type_str(find_dhcp_option('message-type', None, dhcp))
            if message_type == 'nak':
                return 'fail' # TODO Check that from right server?
            
            if message_type != 'ack':
                print "Ignoring packet based on unexpected message_type %s"%message_type
                return False

            lease_time = find_dhcp_option('lease_time', None, dhcp)
            if not lease_time:
                print "Ignoring packet with no lease_time"
            lease_time = lease_time[0] 
            renewal_time = find_dhcp_option('renewal_time', (random.uniform(0.45, 0.55) * lease_time, ), dhcp)[0]
            rebind_time = find_dhcp_option('rebinding_time', (random.uniform(0.825, 0.925) * lease_time, ), dhcp)[0]
            #lease_time = 10
            #rebind_time = 2
            #renewal_time = 1
            ud.dhcp.lease.timeout_time['BOUND'] = renewal_time * 0.99 + self.send_time
            ud.dhcp.lease.timeout_time['RENEW'] = rebind_time * 0.99 + self.send_time
            ud.dhcp.lease.timeout_time['REBIND'] = lease_time * 0.99 + self.send_time


            # FIXME Read other options here.
            return 'success'

        except:
            traceback.print_exc()
            print "Excepiton validating packet."
            return False



class NoLink(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['bound', 'init', 'init_reboot'])

    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.binding_publisher.set(None)
        ud.dhcp.hwaddr = yield async_helpers.wait_for_state(ud.dhcp.link_addr_state, lambda x: x != False)
        network_id = "" # Include MAC, network_id
        if network_id not in ud.dhcp.leases:
            ud.dhcp.leases[network_id] = DhcpLease()
        ud.dhcp.lease = ud.dhcp.leases[network_id]
        returnValue('init')



class Init(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['done', 'nolink'])

    def execute_async(self, ud):
        ud.dhcp.binding_publisher.set(None)
        ud.dhcp.start_socket()
        return 'done'

ETHER_BCAST='ff:ff:ff:ff:ff:ff'
IP_BCAST='255.255.255.255'
IP_ZERO='0.0.0.0'

class Exchange(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['success', 'fail', 'nolink'])

    def send(self, ud):
        hwbytes = scapy.mac2str(ud.dhcp.hwaddr)

        # Prepare options
        options = [
                ("message-type", self.message_type), 
                ("param_req_list", 
                    chr(scapy.DHCPRevOptions["renewal_time"][0]),
                    chr(scapy.DHCPRevOptions["rebinding_time"][0]),
                    chr(scapy.DHCPRevOptions["lease_time"][0]),
                    chr(scapy.DHCPRevOptions["subnet_mask"][0]),
                    chr(scapy.DHCPRevOptions["router"][0]),
                    )
                ]
        if self.type in ["REQUEST", "REBOOT", ]:
            options.append(('requested_addr', ud.dhcp.lease.public_config['ip']))
        if self.type in ["REQUEST", ]:
            options.append(('server_id', ud.dhcp.lease.server_ip))
        options.append('end')
        pkt = scapy.DHCP(options=options)

        # Prepare BOOTP
        pkt = scapy.BOOTP(chaddr=[hwbytes], xid=ud.dhcp.lease.xid)/pkt
        if self.type in [ "RENEW", "REBIND", ]:
            pkt.ciaddr = ud.dhcp.lease.public_config['ip'] 

        # Prepare UDP/IP
        pkt = scapy.IP(src=pkt.ciaddr, dst=IP_BCAST)/scapy.UDP(sport=68, dport=67)/pkt
        if self.type in [ "RENEW" ]:
            pkt.dst = ud.dhcp.lease.server_ip

        # Prepare Ethernet
        pkt = scapy.Ether(src=ud.dhcp.hwaddr, dst=ETHER_BCAST)/pkt
        if self.type in [ "RENEW" ]:
            pkt.dst = ud.dhcp.lease.server_mac
        #print "Out:", repr(scapy.Ether(str(pkt)))
        ud.dhcp.socket.port.send(str(pkt))
                    
    def validate_common(self, ud, pkt, ip, udp, bootp, dhcp): 
        # Should we be receiving this packet?
        if pkt.dst != ud.dhcp.hwaddr and pkt.dst != ETHER_BCAST:
            print "Discarding packet based on destination MAC: %s != %s"%(pkt.dst, ud.dhcp.hwaddr)
            return False

        # Does the xid match?
        if pkt.xid != ud.dhcp.lease.xid:
            print "Discarding packet based on xid: %i != %i"%(pkt.xid, ud.dhcp.lease.xid)
            return False
       
        # TODO Check that from right server?
        
        return True

    @inlineCallbacks
    def execute_async(self, ud):
        # Parameters that will depend on the state.
        timeout = async_helpers.Timeout(self.init_timeouts(ud))

        # Make sure we aren't discarding incoming dhcp packets.
        ud.dhcp.socket.set_discard(False)

        # Generate an xid for the exchange if necessary.
        self.init_xid(ud)

        while True:
            # Send a request packet
            try:
                self.send_time = time.time()
                self.send(ud)
            except:
                traceback.print_exc()
                returnValue('fail')

            # How long to wait before retry
            interval = self.get_next_retry(ud)

            while True:
                # Wait for an event
                events = yield async_helpers.select(
                        async_helpers.StateCondition(ud.dhcp.link_addr_state, lambda x: x == False), 
                        ud.dhcp.socket, 
                        async_helpers.Timeout(interval),
                        timeout)

                if 0 in events: # Lost link
                    returnValue('nolink')
    
                if 1 in events: # Got packet
                    pkt = scapy.Ether(ud.dhcp.socket.recv())
                    #print "In:", repr(pkt)

                    result = self.validate(ud, pkt)
                    if result:
                        returnValue(result)
    
                if 2 in events:
                    break
    
                if 3 in events:
                    returnValue('fail')



class Rebooting  (Exchange, ExchangeRetryExponentialBackoff, ExchangeRequest ): type = "REBOOT"
class Selecting  (Exchange, ExchangeRetryExponentialBackoff, ExchangeDiscover): type = "SELECT"
class Requesting (Exchange, ExchangeRetryExponentialBackoff, ExchangeRequest ): type = "REQUEST"
class Renewing   (Exchange, ExchangeRetryHalve,              ExchangeRequest ): type = "RENEW"
class Rebinding  (Exchange, ExchangeRetryHalve,              ExchangeRequest ): type = "REBIND"



class Error(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['done', 'nolink'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.binding_publisher.set(None)
        ud.dhcp.error_event.trigger()
        ud.dhcp.socket.set_discard(True)
        events = yield async_helpers.select(
                async_helpers.StateCondition(ud.dhcp.link_addr_state, lambda x: x == False),
                async_helpers.Timeout(ud.dhcp.error_timeout)
                )

        if 0 in events:
            returnValue('nolink')

        returnValue('done')


class Bound(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['timeout', 'nolink'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.socket.set_discard(True)
        ud.dhcp.binding_publisher.set(ud.dhcp.lease.public_config)
        events = yield async_helpers.select(
            async_helpers.StateCondition(ud.dhcp.link_addr_state, lambda x: x == False),
            async_helpers.Timeout(ud.dhcp.lease.timeout_time['BOUND'] - time.time()),
            )

        if 0 in events:
            returnValue('nolink')

        returnValue('timeout')


def dhcp_client(iface):
    sm = smach.StateMachine(outcomes=[], input_keys=['dhcp'])
    smadd = smach.StateMachine.add
    with sm:
        smadd('NOLINK',      NoLink(),      transitions = {'bound'  :'BOUND', 'init':'INIT', 'init_reboot':'INIT_REBOOT'})
        smadd('INIT_REBOOT', Init(),        transitions = {'done'   :'REBOOT',                   'nolink':'NOLINK'})
        smadd('REBOOT',      Rebooting(),   transitions = {'success':'BOUND',   'fail':'INIT',   'nolink':'NOLINK'})
        smadd('INIT',        Init(),        transitions = {'done'   :'SELECT',                   'nolink':'NOLINK'})
        smadd('SELECT',      Selecting(),   transitions = {'success':'REQUEST', 'fail':'ERROR',  'nolink':'NOLINK'})
        smadd('REQUEST',     Requesting(),  transitions = {'success':'BOUND',   'fail':'ERROR',  'nolink':'NOLINK'})
        smadd('BOUND',       Bound(),       transitions = {'timeout':'RENEW',                    'nolink':'NOLINK'})
        smadd('RENEW',       Renewing(),    transitions = {'success':'BOUND',   'fail':'REBIND', 'nolink':'NOLINK'})
        smadd('REBIND',      Rebinding(),   transitions = {'success':'BOUND',   'fail':'INIT',   'nolink':'NOLINK'})
        smadd('ERROR',       Error(),       transitions = {'done'   :'INIT',                     'nolink':'NOLINK'})

    ud = smach.UserData()
    ud.dhcp = DhcpData(iface)
#    def shutdown(value):
#        reactor.fireSystemEvent('shutdown')
#    def ignore_eintr(error):
#        if error.type == IOError:
#            import errno
#            if error.value.errno == errno.EINTR:
#                return None
#        return error
    sm.execute_async(ud)#.addCallback(shutdown).addErrback(ignore_eintr)
    return ud.dhcp

if __name__ == "__main__":
    import sys
    from twisted.internet import reactor

    if len(sys.argv) != 2:
        print "usage: dhcp.py <interface>"
        sys.exit(1)

    iface = sys.argv[1]

    dhcp_client(iface)
    reactor.run()    
