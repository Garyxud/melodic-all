#! /usr/bin/env python

# TODO To cleanup: 
# - Mark on ping packets in not being used.
# - Most gateway packets should go through the tunnel.
# - Are we black-holing packets properly when there is nowhere to send them?
# - We could trigger a restrategize as soon as a link goes away.
# - Need to add priorities to different interfaces (wan better than 610n on
# pr2)
# - Refactor so that the link/address state machine is cleaner.
# - Make sure we don't crash if logging fail.
# - Add error checking to logging.

import subprocess
import os
import os.path      
import atexit
import select         
import time
import sys
import threading
import signal
import ipaddr
import network_monitor_udp.udpmoncli
import pythonwifi.iwlibs
import traceback
import errno
import yaml
import math
import inspect
import fcntl
import socket
import logging
import logging.handlers

null_file = open('/dev/null')
                
LOCAL_RULE=50
FIRST_IFACE_RULE=100
TUNNEL_RULE2=75
TUNNEL_RULE=150
BLACKHOLE_BASESTATION_RULE=175
DEFAULT_RULE=200
BLOCK_NON_TUNNEL_RULE=250
        
NOLINK = -2
NOIP = -1
NOCHECK = 0
        
STATUSES = { 
    NOLINK : "Establishing Link", 
    NOIP : "Getting an IP address", 
    NOCHECK : "Ready"
}
             
# Set up logging

class LoggerStream:
    def __init__(self, func):
        self.lock = threading.Lock()
        self.buffer = ""
        self.func = func

    def write(self, str):
        with self.lock:
            self.buffer += str
            while True:
                pos = self.buffer.find('\n')
                if pos == -1:
                    break
                self.func(self.buffer[0:pos])
                self.buffer = self.buffer[pos+1:]

    def flush(self):
        if self.buffer:
            with self.lock:
                self.func(self.buffer)

logdir = '/var/log/roam'

try:
    os.makedirs(logdir)
except:
    pass
logfilecount = 10

file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

console_logger = logging.getLogger('console')
console_logger.setLevel(logging.DEBUG)
strategy_logger = logging.getLogger('console.strategy')
strategy_logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
console_formatter = logging.Formatter('%(message)s')
console_handler.setFormatter(console_formatter)
console_logger.addHandler(console_handler)

console_file_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'console-output.log'), when='midnight', backupCount=logfilecount)
console_file_handler.setFormatter(file_formatter)
console_file_handler.setLevel(logging.DEBUG)
console_logger.addHandler(console_file_handler)

all_logger = logging.getLogger('')
all_logger_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'all.log'), when='midnight', backupCount=logfilecount)
all_logger_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
all_logger_handler.setFormatter(all_logger_formatter)
all_logger_handler.setLevel(logging.DEBUG)
all_logger.addHandler(all_logger_handler)

# Redirect stdout and stderr to the logging system
sys.stdout = LoggerStream(console_logger.info)
sys.stderr = LoggerStream(console_logger.error)
strategy_str = LoggerStream(strategy_logger.info)

def log_time_string(t):
    return time.strftime("%a, %d %b %Y %H:%M:%S.", time.localtime(t))+("%.6f"%t).split('.')[1]

def flushiprule(rule_prio, leave_one = False):
    rules = RunCommand('ip', 'rule').stdout.split('\n')
    prefix = "%i:"%rule_prio
    count = 0
    for line in rules:
        if line.startswith(prefix):
            count += 1
    if leave_one:
        count -= 1
    for i in range(0, count):
        System('ip rule del priority %i'%rule_prio)

def safe_shutdown(method, *args, **nargs):
    try:
        method(*args, **nargs)
    except Exception, e:
        print "Caught exception while shutting down instance.", method
        traceback.print_exc(10)
        print

class System:
    def __init__(self, arg):
        #print arg
        self.errcode = subprocess.call(arg.split(), stdout = null_file, stderr = null_file, close_fds = True)
        #self.errcode = os.system(arg + " 2>&1 > /dev/null")

class RunCommand:
    def __init__(self, *args):
        proc = subprocess.Popen(list(args), stdout = subprocess.PIPE, stderr = subprocess.PIPE, close_fds = True)
        (self.stdout, self.stderr) = proc.communicate()

class CommandWithOutput(threading.Thread):
    def __init__(self, args, name):
        self.restart_delay = 0.2
        threading.Thread.__init__(self, name = name)
        #logname = os.path.join(logdir, '%s.log'%name)
        #try:
        #    os.makedirs(logdir)
        #except OSError, e:
        #    if e.errno != errno.EEXIST:
        #        raise
        #print "Creating log file:", logname
        #self.log = open(os.path.join(logname), 'a')
        #self.log.write("\n\n\nStarting new session...\n")
        self.logger = logging.getLogger(name)
        self.console_logger = logging.getLogger('console.%s'%name)
        logger_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'%s.log'%name), when='midnight', backupCount=logfilecount)
        logger_handler.setFormatter(file_formatter)
        self.logger.addHandler(logger_handler)
        self.console_logger.addHandler(logger_handler)
        self.logger.setLevel(logging.DEBUG)
        self.console_logger.setLevel(logging.DEBUG)
        logger_handler.setLevel(logging.DEBUG)
        self.proc_args = args
        self.start_proc()
        self.running = True
        self.start()
                            
    def start_proc(self):
        try:
            self.proc = subprocess.Popen(self.proc_args, stdout = subprocess.PIPE, stderr = subprocess.STDOUT, close_fds = True)
            flags = fcntl.fcntl(self.proc.stdout, fcntl.F_GETFL)
            fcntl.fcntl(self.proc.stdout, fcntl.F_SETFL, flags| os.O_NONBLOCK)
            self.child_restart()
        except OSError:
            self.console_logger.fatal("Error trying to run: %s"%(" ".join(self.proc_args)))
            raise KeyboardInterrupt

    def child_restart(self):
        pass # Can be overridden by derived classes.

    def run(self):
        read_buffer = {}
        try:
            while True:
                (rd, wr, err) = select.select([self.proc.stdout], [], [], 0.2)
                if not self.running:
                    #print "Exiting CommandWithOutput", self.proc.pid, self.proc_args
                    try:
                        self.proc.send_signal(signal.SIGINT)
                    except OSError, e:
                        if str(e).find('[Errno 3]') == -1:
                            raise
                    #print "Starting Communicate", self.proc_args
                    try:
                        self.proc.communicate()
                    except IOError:
                        pass
                    #print "Ending Communicate", self.proc_args
                    return
                for fd in rd:
                    #print >> sys.stderr, "About to read"
                    try:
                        newdata = fd.read()
                    except IOError:
                        newdata = ""
                    #print >> sys.stderr, "Done read", newdata
                    if len(newdata) == 0: 
                        self.proc.kill()
                        self.proc.communicate()
                        time.sleep(self.restart_delay)
                        if not self.running: 
                            return
                        self.console_logger.info("Process died, restarting: %s"%(" ".join(self.proc_args)))
                        self.start_proc()
                        continue

                    if fd in read_buffer:
                        newdata = read_buffer[fd] + newdata 
                    while True:
                        splitpos = newdata.find('\n')
                        if splitpos == -1:
                            read_buffer[fd] = newdata
                            break
                        line = newdata[0:splitpos]
                        newdata = newdata[splitpos+1:]

                        self.logger.info(line)
                        #now = time.time()
                        #time_str = log_time_string(now)
                        #self.log.write(time_str+": "+line+"\n")
                        #self.log.flush()
                        #sys.stdout.write("%s %s: %s"%(time_str, self.name, line))
                        #sys.stdout.flush()
                        try:
                            self.got_line(line)
                        except Exception, e:
                            self.console_logger.fatal("Caught exception in CommandWithOutput.run: %s"%str(e))
                            raise # FIXME Remove this?
        except: # FIXME Be more persistent?
            traceback.print_exc(10)
            print
            self.console_logger.fatal("Command with output triggering shutdown after exception.")
            os.kill(os.getpid(), signal.SIGINT)
            raise

    def shutdown(self):
        self.running = False
        #print "Shutting down command with output:", self.proc.pid, self.proc_args
        #try:
        #    self.proc.kill()
        #    print "Process killed", self.proc_args
        #except OSError:
        #    print "Process already dead", self.proc_args

class WpaSupplicant(CommandWithOutput):
    def __init__(self, iface, config):
        self.iface = iface
        script = os.path.join(os.path.dirname(__file__), 'wpa_supplicant.sh')
        CommandWithOutput.__init__(self, [script, '-i', iface, '-c', config, '-C', '/var/run/wpa_supplicant', '-dd'], "wpa_supplicant_"+iface)
        self.restart_delay = 1

    def got_line(self, line):
        pass

    def command(self, cmd):
        System('wpa_cli -p /var/run/wpa_supplicant -i %s %s'%(self.iface, cmd))
    
    def restart(self):
        print >> sys.stderr, "Restarting supplicant on %s"%self.iface
        try:
            self.proc.send_signal(signal.SIGINT) 
        except OSError, e:
            if str(e).find('[Errno 3]') == -1:
                raise

class DhcpClient(CommandWithOutput):
    def __init__(self, iface, bound_callback = None, deconfig_callback = None, leasefail_callback = None):
        self.iface = iface
        self.bound_callback = bound_callback
        self.deconfig_callback = deconfig_callback
        self.leasefail_callback = leasefail_callback
        script = os.path.join(os.path.dirname(__file__), 'udhcpc_echo.sh')
        udhcpc_script = os.path.join(os.path.dirname(__file__), 'udhcpc.sh')
        CommandWithOutput.__init__(self, [udhcpc_script, '-s', script, '-i', iface, '-f'], "udhcpc_"+iface)
        self.restart_delay = 0
  
    def got_line(self, line):
        boundprefix = "UDHCPC: bound "
        if line.startswith(boundprefix):
            [iface, ip, netmask, gateway] = line[len(boundprefix):].split() # FIXME may have more than one router
            if self.bound_callback is not None:
                self.bound_callback(iface, ip, netmask, gateway)

        deconfigprefix = "UDHCPC: deconfig "
        if line.startswith(deconfigprefix):
            [iface] = line[len(deconfigprefix):].split()
            if self.deconfig_callback is not None:    
                self.deconfig_callback(iface)
        
        leasefailprefix = "UDHCPC: leasefail "
        if line.startswith(leasefailprefix):
            [iface] = line[len(leasefailprefix):].split()
            if self.leasefail_callback is not None:    
                self.leasefail_callback(iface)
    
    def renew(self):
        # Using SIGINT here because until ralink adapters are first brought up
        # they don't have a configured MAC address, which prevents udhcpc from
        # ever working.
        self.proc.send_signal(signal.SIGINT) 
    
    def release(self):
        try:
            self.proc.send_signal(signal.SIGUSR2)
        except OSError, e:
            if str(e).find('[Errno 3]') == -1:
                raise

class NetlinkMonitor(CommandWithOutput):
    def __init__(self):
        self.lock = threading.RLock()
        self.link_callbacks = {}
        self.addr_callbacks = {}
        self.addr_state = {}
        self.link_state = {}
        self.cur_iface = None
        self.deleted = None
        CommandWithOutput.__init__(self, ['ip', 'monitor', 'link', 'addr'], 'ip_monitor')
# FIXME FIXME Blaise is here.

    def child_restart(self):
        time.sleep(0.2) # Limit race conditions on getting the startup state.
        current_state = RunCommand('ip', 'addr')
        with self.lock:
            old_cur_iface = self.cur_iface
            old_deleted = self.deleted
            for line in current_state.stdout.split('\n'):
                self.got_line(line)
            self.deleted = old_deleted
            self.cur_iface = old_cur_iface

    def register_link(self, iface, callback):
        if ':' in iface:
            iface = iface.split(':')[0] # Handle aliases.
        if iface not in self.link_callbacks:
            self.link_callbacks[iface] = []
        with self.lock:
            self.link_callbacks[iface].append(callback)
            if iface not in self.link_state:
                self.link_state[iface] = False
            callback(self.link_state[iface])

    def register_addr(self, iface, callback):
        print "register_addr", iface
        if iface not in self.addr_callbacks:
            self.addr_callbacks[iface] = []
        with self.lock:
            self.addr_callbacks[iface].append(callback)
            if iface not in self.addr_state:
                self.addr_state[iface] = [None, None]
            callback(*self.addr_state[iface])

    def got_line(self, line):
        with self.lock:
            try:
                # Figure out the interface, and whether this is a delete
                # event.
                
                #print "*****************Got line", line
                if len(line) == 0 or (line[0] == ' ' and self.cur_iface == None):
                    return
                tokens = line.rstrip().split()
                if line[0] != ' ':
                    if tokens[0] == 'Deleted':
                        self.deleted = True
                        tokens.pop(0)
                    else:
                        self.deleted = False
                    self.cur_iface = tokens[1].rstrip(':')
                    tokens.pop(0)
                    tokens.pop(0)

                # Find the link state.
                try:
                    state_idx = tokens.index('state')
                    state = tokens[state_idx + 1]
                    link_state = state != 'DOWN' # Use DOWN because tun shows up as UNKNOWN
                    if self.cur_iface not in self.link_state or self.link_state[self.cur_iface] != link_state:
                        self.link_state[self.cur_iface] = link_state
                        if self.cur_iface in self.link_callbacks:
                            for callback in self.link_callbacks[self.cur_iface]:
                                callback(link_state)
                        print "********************Link change", self.cur_iface, link_state # FIXME Remove this.
                    #else:
                    #    print "********************No link change", self.cur_iface, link_state
                except ValueError:
                    #print "********************No link", self.cur_iface
                    pass
                    
                    
                # Find the address.
                try:
                    addr_idx = tokens.index('inet') 
                    if self.deleted:
                        addr_state = [None, None]
                    else:
                        addr_state = tokens[addr_idx + 1].split('/')
                    full_iface = tokens[-1]
                    if full_iface not in self.addr_state or self.addr_state[full_iface] != addr_state:
                        print "********************Address change", full_iface, addr_state # FIXME Remove this.
                        self.addr_state[full_iface] = addr_state
                        if full_iface in self.addr_callbacks:
                            for callback in self.addr_callbacks[full_iface]:
                                callback(*addr_state)
                    #else:
                    #    print "********************No addr change", self.cur_iface, addr_state
                except ValueError:
                    #print "********************No addr", self.cur_iface
                    pass
            except Exception, e:
                print "Caught exception in NetlinkMonitor.run:", e
                traceback.print_exc(10)
                print

class NetworkConnection:
    def __init__(self, name, iface, config, tableid, pingtarget):
        self.timeout_time = 1e1000
        self.status = NOLINK
        if 'name' in config:
            self.name = config['name']
        else:
            self.name = name
        self.iface = iface
        self.tableid = tableid
        self.shutting_down = False
        self.bssid = "NoLink"
        self.priority = config['priority'] if 'priority' in config else 0

        self.udp_monitor = network_monitor_udp.udpmoncli.MonitorClient([.2], pingtarget, 20, 32, True)
        netlink_monitor.register_addr(self.iface, self.addrchange)

    def addrchange(self, addr, netbits):
        if addr:
            self.status = NOCHECK
            # It is critical that set_routes be called to set up the routes
            # before monitor_start. Otherwise, the UDP connection will get
            # started with the wrong route, and conntrack will cache the
            # incorrect route, preventing the routing table change from
            # being properly considered.
            self.set_routes(addr, netbits) 
            try:
                print 'start_monitor', addr
                self.udp_monitor.start_monitor((addr, 0))
            except:
                pass # Address gets set twice (first address then netmask). The first time often results in a bind failure.
        elif self.status == NOCHECK:
            print 'stop_monitor'
            self.status = NOIP
            self.udp_monitor.stop_monitor()
    
    def update1(self):
        self.monitor_output = self.udp_monitor.get_smart_bins(1)
        (bins, latency1, latency2) = self.monitor_output
        
        self.diags = [] 
        
        self.diags.append(("Link status", (STATUSES[self.status] if self.status in STATUSES else "unknown")))
   
        self.diags.append(("Packets under 200ms latency (%)", 100 * bins[0]))
        self.diags.append(("Average round trip latency (ms)", "%.1f"%(latency1*1000)))
        self.diags.append(("Average latency for packets under 200ms (ms)", "%.1f"%(latency2*1000)))

        return (bins, latency1, latency2)

    def update2(self):
        (bins, latency1, latency2) = self.monitor_output
        
        if self.status < 0:
            self.goodness = self.status
            self.reliability = self.status
        else:
            self.goodness = 100 * bins[0] - latency2 # Goodness is how many packets made it through then average latency.
        
        self.diags.append(("Goodness:", self.goodness))
        self.diags.append(("Reliability:", self.reliability))

        if self.status < 0:
            self.diag_summary = STATUSES[self.status]
        else:
            self.diag_summary = "Connected (goodness %f, reliability %f)"%(self.goodness, self.reliability)

    def shutdown(self):
        self.shutting_down = True
        safe_shutdown(self.udp_monitor.shutdown)

class StaticRoute(NetworkConnection):
    def __init__(self, route, config, tableid, pingtarget):
        self.gateway, iface = route.split('@')
        NetworkConnection.__init__(self, route, iface, config, tableid, pingtarget)
        netlink_monitor.register_link(self.iface, self.linkchange)

    def increase_timeout(self, delay = 1e1000):
        pass
    
    def set_timeout(self, delay):
        pass
    
    def decrease_timeout(self, delay):
        pass
        
    def set_routes(self, addr, netbits):
        flushiprule(self.tableid)
        System("ip rule add priority %i from %s table %i"%(self.tableid,addr,self.tableid))
        System("ip route flush table %i"%self.tableid)
        try:
            gatewayip = socket.gethostbyname(self.gateway)
        except socket.gaierror, e:
            # TODO Should I bail out at this point?
            print >> sys.stderr, "Error resolving gateway host %s. Connection self.name will not work."%self.gateway
            return
        System("ip route add table %i default dev %s via %s src %s onlink"%(self.tableid, self.iface, gatewayip, addr))
        System("ip route add table %i %s dev %s src %s"%(self.tableid, gatewayip, self.iface, addr))
                    
    def linkchange(self, up):
        if not up:
            self.bssid = "NoLink"
            self.status = NOLINK
        else:
            self.bssid = "Wired"
            if self.status == NOLINK:
                self.status = NOIP

    def update(self):
        self.update1()
        self.reliability = 100        
        self.update2()

class DhcpInterface(NetworkConnection):
    def __init__(self, iface, config, tableid, pingtarget):
        self.is_bound = False
        NetworkConnection.__init__(self, iface, iface, config, tableid, pingtarget)
        self.start_over_timeout = 15
        self.dhcp_timeout = 7
        self.timeout_time = 0
        self.increase_timeout(self.start_over_timeout)
        self.timed_out_time = 0

        self.dhclient = DhcpClient(iface, self.bound, self.deconfigured, self.leasefail)
        self.startover()
    
    def update1(self):
        time_to_timeout = self.timeout_time - time.time()
        if time_to_timeout < 0:
            print "Interface", self.name, "timed out."
            self.timed_out_time = time.time()
            self.startover()

        NetworkConnection.update1(self)

        self.diags.append(("Time to interface timeout (s)", time_to_timeout))

    def linkchange(self, up):
        # We want to be sure some timeout is set here.
        self.set_timeout(self.start_over_timeout)
        #print "linkchange", self.name, address
        if up:
            if self.status == NOLINK:
                self.bssid = "Wired"
                self.status = NOIP
                self.set_timeout(self.dhcp_timeout)
                self.dhclient.renew()
        else:
            if self.status != NOLINK:
                self.bssid = "NoLink"
                self.status = NOLINK
                self.udp_monitor.stop_monitor()
                self.dhclient.release()
        pass 
  
    def set_routes(self, addr, netbits):
        if not self.is_bound:
            return
        flushiprule(self.tableid)
        #System("ip rule add priority %i from %s table %i fwmark 2"%(self.tableid,ip,self.tableid))
        System("ip rule add priority %i from %s table %i"%(self.tableid,addr,self.tableid))
        System("ip route flush table %i"%self.tableid)
        System("ip route add table %i default dev %s via %s src %s onlink"%(self.tableid, self.iface, self.gateway, addr))
        net = ipaddr.IPv4Network("%s/%s"%(addr,netbits))
        #self.network = "%s/%s"%(net.network,net.prefixlen)
        print("ip route add table %i %s/%s dev %s src %s"%(self.tableid, net.network, net.prefixlen, self.iface, addr))
        System("ip route add table %i %s/%s dev %s src %s"%(self.tableid, net.network, net.prefixlen, self.iface, addr))
    
    def bound(self, iface, ip, netmask, gateway):
        print "Bound!", self.status
        #if self.is_bound == True:
        #    self.deconfigured(iface)
        self.increase_timeout(self.start_over_timeout)
        self.is_bound = True
        self.ip = ip
        self.netmask = netmask
        self.gateway = gateway
        
        # Configure the address on the interface, and make sure ARP is up
        # to date on peers.
        System("ifconfig %s 0.0.0.0"%iface)
        net = ipaddr.IPv4Network("%s/%s"%(ip, netmask))
        System("ip addr add %s/%s dev %s"%(ip, net.prefixlen, iface))
        #System("ifconfig %s %s netmask %s"%(iface,ip,netmask))
        System("arping -q -c 1 -A -I %s %s"%(iface,ip))
        ConfigureRpFilter(iface) # FIXME Do this here in case the interface was unplugged and replugged.
        print "Bound done!", self.status

    def startover(self): # Deconfigure the interface and start over from scratch
        flushiprule(self.tableid)
        System("ifconfig %s 0.0.0.0 down"%self.iface) # This should clear all routing table entries.
        System("ifconfig %s up"%self.iface)
        self.set_timeout(self.start_over_timeout)

    def leasefail(self, iface):
        if self.status > NOLINK:
            self.startover()

    def deconfigured(self, iface):
        if self.is_bound:
            self.is_bound = False
            flushiprule(self.tableid)
            System("ifconfig %s 0.0.0.0 down"%iface) # This should clear all routing table entries.
            System("ifconfig %s up"%iface)

    def shutdown(self):
        NetworkConnection.shutdown(self)
        System("ifconfig %s down"%self.iface) # This should clear all routing table entries.
        safe_shutdown(self.dhclient.shutdown)

    def decrease_timeout(self, delay):
        # Call this if things are not doing well, and you want to pull in the timeout horizon.
        self.timeout_time = min(time.time() + delay, self.timeout_time)

    def increase_timeout(self, delay = 1e1000):
        # Call this if things are doing well and you want to push back the timeout horizon.
        self.timeout_time = max(time.time() + delay, self.timeout_time)
    
    def set_timeout(self, delay):
        # Call this if you want to set the timeout exactly.
        self.timeout_time = time.time() + delay
           
class WirelessInterface(DhcpInterface):
    def __init__(self, iface, config, tableid, pingtarget):
        self.startover_count = 0
        
        DhcpInterface.__init__(self, iface, config, tableid, pingtarget)

        self.wifi = pythonwifi.iwlibs.Wireless(iface)
        self.iwinfo = pythonwifi.iwlibs.WirelessInfo(iface)
        
        self.supplicant = WpaSupplicant(iface, config['wpa_config'])
        self.initialized = True
        netlink_monitor.register_link(iface, self.linkchange)
    
    def linkchange(self, up):
        if not self.initialized:
            return
        
        if up:
            self.startover_count = 0
        # if self.status == NOLINK and up:
        #     self.supplicant.command('reassociate')
        #     print "************** reassociate"
        DhcpInterface.linkchange(self, up)

    def startover(self):
        if self.startover_count > 1:
            self.startover_count = 0
            self.supplicant.restart()
        else:
            self.startover_count += 1

        DhcpInterface.startover(self)  
    
    def update(self):
        self.update1()
        
        try:
            self.essid = self.wifi.getEssid()
            self.diags.append(('ESSID', self.essid))
        except Exception, e:
            if self.status != NOLINK:
                traceback.print_exc(10)
                print
            self.diags.append(('ESSID', 'Error collecting data.'))
            self.essid = "###ERROR-COLLECTING-DATA###"

        try:
            self.bssid = self.wifi.getAPaddr()
            self.diags.append(('BSSID', self.bssid))
        except Exception, e:
            if self.status != NOLINK:
                traceback.print_exc(10)
                print
            self.bssid = "00:00:00:00:00:00"
            self.diags.append(('BSSID', 'Error collecting data.'))

        try:
            self.wifi_txpower = 10**(self.wifi.wireless_info.getTXPower().value/10.)
            self.diags.append(('TX Power (mW)', self.wifi_txpower))
            self.wifi_txpower = "%.1f mW"%self.wifi_txpower
        except Exception, e:
            if str(e).find("Operation not supported") == -1 and self.status != NOLINK:
                traceback.print_exc(10)
                print
            self.diags.append(('TX Power (mW)', 'Error collecting data.'))
            self.wifi_txpower = "unknown" 

        try:
            self.wifi_frequency = self.wifi.wireless_info.getFrequency().getFrequency()
            self.diags.append(('Frequency (Gz)', "%.4f"%(self.wifi_frequency/1e9)))
        except Exception, e:
            if self.status != NOLINK:
                traceback.print_exc(10)
                print
            self.wifi_frequency = 0
            self.diags.append(('Frequency', 'Error collecting data.'))

        got_stats = False
        if self.status != NOLINK:
            try:
                stat, qual, discard, missed_beacon = self.wifi.getStatistics()
                max_quality = self.wifi.getQualityMax().quality
                quality = qual.quality * 100 / max_quality
                self.diags.append(('Quality', quality))
                self.diags.append(('Signal (dB)', qual.siglevel))
                self.diags.append(('Noise (dB)', qual.nlevel))
                self.diags.append(('SNR (dB)', qual.siglevel - qual.nlevel))
                self.wifi_signal = qual.siglevel
                self.wifi_noise = qual.nlevel
                self.wifi_quality = quality
                self.reliability = quality
                got_stats = True
            except Exception, e:
                print "Error getting wireless stats on interface %s: %s"%(self.iface, str(e))
        
        if not got_stats:
            #print self.name, "could not collect wireless data", e
            print
            self.reliability = 0
            self.wifi_quality = -1
            self.wifi_noise = 1e1000
            self.wifi_signal = -1e1000
            for s in [ 'Quality', 'Signal', 'Noise' ]:
                if self.status != NOLINK:
                    self.diags.append((s, 'Error collecting data.'))
                else:
                    self.diags.append((s, 'Unknown'))

        self.wifi_rate = None
        if self.status != NOLINK:
            try:
                self.wifi_rate = self.wifi.wireless_info.getBitrate().value
            except:
                pass
            if self.wifi_rate is None:
                try:
                    self.wifi_rate = self.iwinfo.getBitrate().value
                except:
                    pass
        if self.wifi_rate is not None:    
            self.diags.append(('TX Rate (Mbps)', self.wifi_rate / 1e6))
            self.wifi_rate = self.wifi._formatBitrate(self.wifi_rate)
        else:
            if self.status != NOLINK:
                print "Unable to determine TX rate on interface", self.iface
                self.diags.append(('TX Rate (Mbps)', 'Error collecting data.'))
            else:
                self.diags.append(('TX Rate (Mbps)', 'Unknown'))
            self.wifi_rate = "Unknown"
        
        self.update2()

    def shutdown(self):
        DhcpInterface.shutdown(self)
        safe_shutdown(self.supplicant.shutdown)

class WiredInterface(DhcpInterface):
    def __init__(self, iface, config, tableid, pingtarget):
        self.initialized = False
        DhcpInterface.__init__(self, iface, config, tableid, pingtarget)
        self.initialized = True
        netlink_monitor.register_link(iface, self.linkchange)
    
    def linkchange(self, up):
        if up != (self.status != NOLINK) and self.initialized and time.time() - self.timed_out_time > 10:
            os.system('beep')
        DhcpInterface.linkchange(self, up)
    
    def update(self):
        self.update1()
        self.reliability = 100        
        self.update2()

    def shutdown(self):
        self.initialized = False
        DhcpInterface.shutdown(self)

# class PacketMarker:
#     def __init__(self, basestation):
#         print "Initializing PacketMarker."
#         self.basestation = basestation
#         self.rules = []
#         # Packets for the VPN get a mark of 1.
#         self.rules.append("OUTPUT -t mangle -d %s -p udp --dport 1194 -j MARK --set-mark 1"%basestation)
#         # Packets for the ping check get a mark of 2.
#         self.rules.append("OUTPUT -t mangle -d %s -p udp --dport 6868 -j MARK --set-mark 2"%basestation)
#         self.playrules("-D", True)
#         self.playrules("-A", False)
#         print "PacketMarker initialized."

#     def playrules(self, command, repeat):
#         for rule in self.rules:
#             while True:
#                 if System("iptables %s %s"%(command, rule)).errcode:
#                     break
#                 if not repeat:
#                     break

#     def shutdown(self):
#         self.playrules("-D", False)

class RoutingRules:
    def __init__(self, numinterfaces, localnets, tuniface, basestation): 
        self.numinterfaces = numinterfaces
        self.tuniface = tuniface
        self.flushall()

        System("ip rule add priority %i to %s blackhole"%(BLACKHOLE_BASESTATION_RULE, basestation))

        #System("ip rule add priority %i fwmark 1 table %i"%(BLOCK_TUNNEL_RULE,BLOCK_NON_TUNNEL_RULE))
        #flushiprule(BLOCK_TUNNEL_RULE, True)
        #System("ip route replace blackhole default table %i src 127.0.0.1"%BLOCK_NON_TUNNEL_RULE)
        #System("ip rule add priority %i table %i"%(BLOCK_NON_TUNNEL_RULE,BLOCK_NON_TUNNEL_RULE))
        #flushiprule(BLOCK_NON_TUNNEL_RULE, True)
        for net in localnets:
            System("ip rule add priority %i to %s lookup main"%(LOCAL_RULE, net))
        System("ip rule add priority %i lookup %i"%(DEFAULT_RULE,DEFAULT_RULE))
        #System("ip rule add priority %i blackhole fwmark 1"%(TUNNEL_RULE2))
        #System("ip rule add priority %i blackhole fwmark 1"%(TUNNEL_RULE))
        netlink_monitor.register_link(self.tuniface, self.refresh)

    def refresh(self, up): # Call this often in case an interface goes away for a bit.
        if up:
            System("ip route replace table %i default dev %s"%(DEFAULT_RULE, self.tuniface))

    def flushall(self):
        # Make sure that all the rules from an earlier run are flushed out.
        for i in range(0,self.numinterfaces):
            flushiprule(FIRST_IFACE_RULE + i)
            flushiprule(TUNNEL_RULE + i)
        flushiprule(DEFAULT_RULE)
        flushiprule(LOCAL_RULE)
        flushiprule(BLACKHOLE_BASESTATION_RULE)
        #flushiprule(TUNNEL_RULE2)
        #flushiprule(BLOCK_TUNNEL_RULE, True)
        #flushiprule(BLOCK_NON_TUNNEL_RULE, True)

        # Make sure that all the tables from an earlier run are flushed out.
        for i in range(0,self.numinterfaces):
            self.flushtable(FIRST_IFACE_RULE + i)
        self.flushtable(DEFAULT_RULE)
        # Don't flush the BLOCK_NON_TUNNEL_RULE table as we want it to persist.
    
    def flushtable(self, tab_id):
        System("ip route flush table %i"%tab_id)

    def shutdown(self):
        self.flushall()

class ConfigureArp:
    def __init__(self):
        System("sysctl net.ipv4.conf.all.arp_filter=1")

class ConfigureRpFilter:
    def __init__(self, iface):
        System("sysctl net.ipv4.conf.%s.rp_filter=0"%iface)

class KillServices:
    def __init__(self):
        System("killall wpa_supplicant udhcpc 2> /dev/null")

class NetworkSelector:
    def __init__(self, config):
        interfaces_config = config['interfaces']
        self.base_station = socket.gethostbyname(config['base_station'])
        pingaddr = (self.base_station, int(config['ping_port']))
        self.tunnel_interface = config['tunnel_interface']
        local_networks = config['local_networks']

        KillServices()
        global netlink_monitor
        netlink_monitor = NetlinkMonitor()
        #self.pkt_marker = PacketMarker(self.base_station)
        ConfigureArp()
        ConfigureRpFilter('all')
        self.routing_rules = RoutingRules(len(interfaces_config), local_networks, self.tunnel_interface, self.base_station)
        self.interfaces = []
        self.active_iface = -1
        self._tunnel_rules = {}
        self._set_tunnel_rule(TUNNEL_RULE, "blackhole")

        i = -1
        for iface in interfaces_config:
            i += 1
            opts = interfaces_config[iface]
            type = opts['type']
            if type == "wireless":
                if 'wpa_config' not in opts and 'wpa_config' in config:
                    opts['wpa_config'] = config['wpa_config']
                self.interfaces.append(WirelessInterface(iface, opts, FIRST_IFACE_RULE + i, pingaddr))
            elif type == "wired":
                self.interfaces.append(WiredInterface(iface, opts, FIRST_IFACE_RULE + i, pingaddr))
            elif type == "static":
                self.interfaces.append(StaticRoute(iface, opts, FIRST_IFACE_RULE + i, pingaddr))
            else:
                raise Exception("Unknown type for an interface: %s"%type)
        
        self.interface_names = [ iface.name for iface in self.interfaces ]

    def update(self):
        # Update interfaces
        for iface in self.interfaces:
            iface.update()
       
        # We use information from the interface that was active for the preceding time slice. 
        self.diags = []
        self.diags.append(('Tunnel Interface', self.tunnel_interface))
        if self.active_iface >= 0:
            act_iface = self.interfaces[self.active_iface]
            self.diags.append(('Active Interface', act_iface.iface ))
            self.diags += act_iface.diags
            if act_iface.goodness > 95:
                self.diag_summary = "Active interface %s running strong"%act_iface.iface
                self.diag_level = 0
            elif act_iface.goodness > 50:
                self.diag_summary = "Active interface %s is lossy"%act_iface.iface
                self.diag_level = 1
            else:
                self.diag_summary = "Active interface %s is very poor"%act_iface.iface
                self.diag_level = 2
        else:
            self.diags.append(('Active Interface', "none"))
            self.diag_summary = 'No active interface'
            self.diag_level = 2

    def prepare_diagnostics(self):
        pass
                    
    def shutdown(self): # Add exception handling so that one failure won't prevent others from shutting down.
        #safe_shutdown(rospy.signal_shutdown, "Node is going down.")
        if hasattr(self, "interfaces"):
            for iface in self.interfaces:
                #print "Shutting down interface %s."%iface.name
                safe_shutdown(iface.shutdown)
        safe_shutdown(netlink_monitor.shutdown)
        #print "Shutting down packet marker."
        #safe_shutdown(self.pkt_marker.shutdown)
        #print "Shutting down routing rules."
        safe_shutdown(self.routing_rules.shutdown)

    def make_active(self, iface):
        if not iface:
            self.active = -1
            return
        self.make_active_multiple([iface])

    def _set_tunnel_rule(self, priority, rule):
        System("ip rule add priority %i %s"%(priority, rule))
        if priority in self._tunnel_rules:
            System("ip rule del priority %i %s"%(priority, self._tunnel_rules[priority]))
        self._tunnel_rules[priority] = rule

    def make_active_non_tunnel(self, iface):
        if not iface:
            self.active = -1
            return
        
        self.active_iface = self.interface_names.index(iface.name)
        self._set_tunnel_rule(TUNNEL_RULE,"table %i"%iface.tableid)

    def make_active_multiple(self, iface):
        n = len(iface)
        if not n:
            self.active_iface = -1
            return
        
        self.active_iface = self.interface_names.index(iface[0].name)
        for i in range(0, n):
            self._set_tunnel_rule(TUNNEL_RULE+i,"table %i to %s"%(iface[i].tableid, self.base_station))
            #System("ip rule add priority %i table %i fwmark 1"%(TUNNEL_RULE2,iface.tableid))
            #System("ip rule del priority %i"%TUNNEL_RULE2)

class SelectionStrategy:
    def __init__(self, config):
        self.ns = NetworkSelector(config)

    def update(self):
        ns = self.ns
        ns.update()
        print >> strategy_str
        print >> strategy_str, log_time_string(time.time())
        self.do_update()
        ns.prepare_diagnostics()

    def shutdown(self):
        safe_shutdown(self.ns.shutdown)

class StatGatherer:
    def __init__(self):
        self.max = -1e1000
        self.min = 1e1000
        self.sum = 0.
        self.count = 0
        self.avg = 0.
        self.dev = 0.
        self.sqrsum = 0.

    def append(self, value):
        self.max = max(self.max, value)
        self.min = min(self.min, value)
        self.sum += value
        self.sqrsum += value * value
        self.count += 1
        self.avg = self.sum / self.count
        self.dev = math.sqrt(self.sqrsum / self.count - self.avg * self.avg)

    def to_string(self):
        return "avg: %5.1f dev: %5.1f min: %5.1s max: %5.1f count: %5i"%(self.avg, self.dev, self.min, self.max, self.count)

class LinkBringupTimeMeasurementStrategy(SelectionStrategy):
    def __init__(self, config):
        SelectionStrategy.__init__(self, config)
        self.start_times = len(self.ns.interfaces) * [time.time()]
        self.stats = [ StatGatherer() for iface in self.ns.interfaces]

    def do_update(self):
        ns = self.ns
        now = time.time()
        for i in range(0, len(ns.interfaces)):
            iface = ns.interfaces[i]
            if iface.goodness >= 90:
                self.stats[i].append(now - self.start_times[i])
                iface.startover()
                self.start_times[i] = now
            print >> strategy_str, "%s %5.0f %5.1fs %4.1fs %s"%(iface.name, iface.goodness, now - self.start_times[i], iface.timeout_time - now, self.stats[i].to_string())
        print >> strategy_str

class LinkStabilityTimeMeasurementStrategy(SelectionStrategy):
    def __init__(self, config):
        SelectionStrategy.__init__(self, config)
        self.is_up = len(self.ns.interfaces) * [ False ]
        self.start_times = len(self.ns.interfaces) * [time.time()]
        self.start_stats = [ StatGatherer() for iface in self.ns.interfaces]
        self.longevity_stats = [ StatGatherer() for iface in self.ns.interfaces ]

    def do_update(self):
        ns = self.ns
        now = time.time()
        for i in range(0, len(ns.interfaces)):
            iface = ns.interfaces[i]
            is_now_up = self.is_up[i]
            if iface.goodness >= 90:
                is_now_up = True
            elif iface.goodness < 0:
                is_now_up = False
            if is_now_up:
                if iface.goodness >= 90:
                    iface.increase_timeout()
                else:
                    iface.decrease_timeout(5)
            if self.is_up[i] != is_now_up:
                (self.start_stats if is_now_up else self.longevity_stats)[i].append(now - self.start_times[i])
                self.start_times[i] = now
                self.is_up[i] = is_now_up
                        
            #print iface.name, ("up  " if is_now_up else "down"), "%5.1fs"%(now - self.start_times[i]), "%4.1fs"%(iface.timeout_time - now), "START", self.start_stats[i].to_string(), "LONGEVITY", self.longevity_stats[i].to_string()
            print >> strategy_str, iface.name, "%5.0f"%iface.goodness, "%5.1fs"%(now - self.start_times[i]), "%4.1fs"%(iface.timeout_time - now), "START", self.start_stats[i].to_string(), "LONGEVITY", self.longevity_stats[i].to_string()

class SimpleSelectionStrategy(SelectionStrategy):
    def __init__(self, config):
        self.best_wireless = -1
        SelectionStrategy.__init__(self, config)
        try:
            self.reliability_thresh = config['reliability_threshold']
        except:
            self.reliability_thresh = 90

    def do_update(self):
        ns = self.ns
        # Get a sorted list of working interfaces.
        iface_with_sort_param = []
        best_bssid = ns.interfaces[self.best_wireless].bssid if self.best_wireless != -1 else None
        for i in ns.interfaces:
            if i.goodness <= 0:
                continue
            bonus = 0
            if self.best_wireless != -1:
                if i == ns.interfaces[self.best_wireless]:
                    bonus += 5
                elif i.bssid != best_bssid:
                    bonus -= 5
            #if i == ns.interfaces[ns.active_iface]:
            #    bonus += 10
            sort_param = i.goodness + i.reliability + i.priority + bonus
            iface_with_sort_param.append((i, sort_param))
        if iface_with_sort_param:
            iface_with_sort_param.sort(key = lambda tuple: tuple[1], reverse = True)
            iface_sorted, _ = zip(*iface_with_sort_param)
        else:
            iface_sorted = []
        
        ns.make_active_multiple(iface_sorted)

        # Figure out the best wireless interface
        iface_types = map(lambda x: x.__class__, iface_sorted)
        try:
            wireless_index = iface_types.index(WirelessInterface)
            self.best_wireless = ns.interfaces.index(iface_sorted[wireless_index])
        except:
            self.best_wireless = -1
        
        # Decide if the other interfaces should go down
        for i in range(0,len(ns.interfaces)):
            interface = ns.interfaces[i]
            if interface.goodness < 0: 
                pass # Card not configured, we are not in charge.
            elif i == self.best_wireless:
                if interface.goodness > 0:
                    # Do not reset the active connection if it is slightly alive.
                    interface.increase_timeout()
            elif interface.reliability < self.reliability_thresh or interface.goodness < 50:
                # Restart unreliable non-active connections.
                interface.decrease_timeout(3)
            else:
                # This non-active interface is doing fine. No timeout.
                interface.increase_timeout()

        # Print active_iface status
        for iface in ns.interfaces:
            try:
                rank = iface_sorted.index(iface) + 1
                if rank == 1:
                    is_active = "active"
                else:
                    is_active = "#%i"%rank
                if self.best_wireless != -1 and iface == ns.interfaces[self.best_wireless]:
                    is_active += ", best wifi"
            except ValueError:
                is_active = ""
            print >> strategy_str, "%10s %5.1f %17s %7.3f %3.0f %s"%(iface.name, (iface.timeout_time - time.time()), iface.bssid, iface.goodness, iface.reliability, is_active)

class AlwaysSwitchSelectionStrategy(SelectionStrategy):
    def __init__(self, config):
        SelectionStrategy.__init__(self, config)

    def do_update(self):
        ns = self.ns
        # Update metrics
        goodnesses = [ i.goodness for i in ns.interfaces ]
        reliabilities = [ i.reliability for i in ns.interfaces ]
        priorities = [ i.priority for i in ns.interfaces ]

        # Pick new active interface
        sort_param = [ sum(tuple) for tuple in zip(goodnesses, reliabilities, priorities) ]
        next_iface = (ns.active_iface + 1) % len(sort_param)
        if sort_param[next_iface] > 180:
          best_iface = next_iface
        else:
          best_sort_param = max(sort_param)
          best_iface = sort_param.index(best_sort_param)
        if goodnesses[best_iface] < 0:
            ns.active_iface = -1
        elif best_iface != ns.active_iface:
            ns.active_iface = best_iface
            ns.make_active(ns.interfaces[ns.active_iface])

        # Decide if the other interfaces should go down
        for i in range(0,len(ns.interfaces)):
            interface = ns.interfaces[i]
            if interface.goodness < 0: 
                pass # Card not configured, we are not in charge.
            elif i == ns.active_iface:
                if interface.goodness > 0:
                    # Do not reset the active connection if it is slightly alive.
                    interface.increase_timeout()
            elif interface.reliability < 90 or interface.goodness < 50:
                # Restart unreliable non-active connections.
                interface.decrease_timeout(3)
            else:
                # This non-active interface is doing fine. No timeout.
                interface.increase_timeout()

        # Print active_iface status
        for iface in ns.interfaces:
            if ns.active_iface != -1 and iface == ns.interfaces[ns.active_iface]:
                is_active = "active"
            else:
                is_active = ""
            print >> strategy_str, "%s %.1f %s %.3f %.0f %s"%(iface.name, (iface.timeout_time - time.time()), iface.bssid, iface.goodness, iface.reliability, is_active)

def main(config_file, strategy, supervisor_function = None):
    if os.getuid() != 0:
       print >> console, "roam.py must be run as root!"
       sys.exit(1)
    with open(config_file, 'r') as f:
       config = yaml.load(f.read())
    try:
        try:
            try:
               s = strategy(config)
            
               while True:
                   s.update()
                   if supervisor_function:
                       try:
                           supervisor_function(s)
                       except:
                           print "Exception in supervisor_function."
                           traceback.print_exc(10)
                           print
                   time.sleep(1)
            
            except KeyboardInterrupt:
                print "Exiting on CTRL-C"
        except:
            traceback.print_exc(10)
            print
            raise
    finally:
        try:
            safe_shutdown(s.shutdown)
        except:
            traceback.print_exc(10)
            print
    
    print
    print "End of main reached. Waiting for threads to shut down."
    time.sleep(0.1)
    while True:    
        threads = threading.enumerate()
        non_daemon = sum(0 if t.daemon else 1 for t in threads)
        if non_daemon == 1:
            break
        print
        print "Remaining threads:", non_daemon, len(threads)
        for t in threads:
            print ("daemon: " if t.daemon else "regular:"), t.name
        time.sleep(1)

if __name__ == "__main__":
    main('roam_config.yaml', SimpleSelectionStrategy)
    #main('roam_config.yaml', LinkBringupTimeMeasurementStrategy)
    #main('roam_config.yaml', LinkStabilityTimeMeasurementStrategy)
        
