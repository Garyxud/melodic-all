import subprocess
import unittest

class TcPortControl:
    def __init__(self, testcase, interface = "lo", port = 12345):
        self.testcase = testcase
        self.interface = interface
        self.port = port
        self.devnull = open('/dev/null', 'w')

    def reset(self):
        subprocess.call(['tc', 'qdisc', 'del', 'dev', self.interface, 'root'], 
                        stdout = self.devnull, stderr = subprocess.STDOUT)
        subprocess.call(['tc', 'qdisc', 'del', 'dev', self.interface, 'ingress'], 
                        stdout = self.devnull, stderr = subprocess.STDOUT)

    def init(self):
        ret = subprocess.call(['tc', 'qdisc', 'add', 'dev', self.interface, 'handle', '1:', 'root', 'htb'], 
                              stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertEqual(ret, 0, "Setting htb qdisc failed")
        ret = subprocess.call(['tc', 'qdisc', 'add', 'dev', self.interface, 'handle', 'ffff:', 'ingress'], 
                              stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertEqual(ret, 0, "Setting ingress qdisc failed")
        ret = subprocess.call(['tc', 'class', 'add', 'dev', self.interface, 'parent', '1:', 'classid', '1:1',
                         'htb', 'rate', '1000Mbps'], stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertEqual(ret, 0, "Adding htb class failed")
        ret = subprocess.call(['tc', 'filter', 'add', 'dev', self.interface, 'protocol', 'ip', 'prio', '1', 
                               'u32', 'match', 'ip', 'dport', str(self.port), '0xffff', 'flowid', '1:1'], 
                              stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertEqual(ret, 0, "Adding destination port " + str(self.port) + " filter failed")

    def set_rate_limit(self, rate):
        ret1 = subprocess.call(['tc', 'qdisc', 'add', 'dev', self.interface, 'parent', '1:1',
                                'tbf', 'rate', str(rate), 'buffer', str(int(rate*625/1e5)),
                                'limit', str(int(rate * 1171875/1e8))], 
                               stdout = self.devnull, stderr = subprocess.STDOUT)
        ret2 = subprocess.call(['sudo', 'tc', 'qdisc', 'change', 'dev', self.interface, 'parent', '1:1',
                                'tbf', 'rate', str(rate), 'buffer', str(int(rate*625/1e5)),
                                'limit', str(int(rate * 1171875/1e8))], 
                               stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertTrue(ret1 == 0 or ret2 == 0,
                                 "Setting the rate limit failed on loopback interface")

    def set_latency_loss(self, latency = 0.0, loss = 0.0):
        ret1 = subprocess.call(['tc', 'qdisc', 'add', 'dev', self.interface, 'parent', '1:1',
                                'netem', 'latency', '%.2fms'%(latency*1000), 'loss', '%.2f%%'%(loss)],
                               stdout = self.devnull, stderr = subprocess.STDOUT)
        ret2 = subprocess.call(['tc', 'qdisc', 'change', 'dev', self.interface, 'parent', '1:1',
                               'netem', 'latency', '%.2fms'%(latency*1000), 'loss', '%.2f%%'%(loss)],
                               stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertTrue(ret1 == 0 or ret2 == 0,
                                 "Setting netem latency and loss parameters failed on loopback interface")

    def set_latency_loss_udp_returnpath(self, latency = 0.0, loss = 0.0):
        subprocess.call(['modprobe', 'ifb'],
                        stdout = self.devnull, stderr = subprocess.STDOUT)
        subprocess.call(['ip', 'link', 'set', 'dev', 'ifb0', 'up'],
                        stdout = self.devnull, stderr = subprocess.STDOUT)
        subprocess.call(['tc', 'qdisc', 'del', 'dev', 'ifb0', 'root'], 
                        stdout = self.devnull, stderr = subprocess.STDOUT)
        subprocess.call(['tc', 'filter', 'add', 'dev', self.interface, 'parent', 'ffff:', 'protocol', 'ip',
                         'u32', 'match', 'ip', 'sport', str(self.port), '0xffff', 'flowid', '1:1', 
                         'action', 'mirred', 'egress', 'redirect', 'dev', 'ifb0'],
                        stdout = self.devnull, stderr = subprocess.STDOUT)

        ret1 = subprocess.call(['tc', 'qdisc', 'add', 'dev', 'ifb0', 'root',
                                'netem', 'latency', '%.2fms'%(latency*1000), 'loss', '%.2f%%'%(loss)],
                               stdout = self.devnull, stderr = subprocess.STDOUT)
        ret2 = subprocess.call(['tc', 'qdisc', 'change', 'dev', 'ifb0', 'root',
                               'netem', 'latency', '%.2fms'%(latency*1000), 'loss', '%.2f%%'%(loss)],
                               stdout = self.devnull, stderr = subprocess.STDOUT)
        self.testcase.assertTrue(ret1 == 0 or ret2 == 0,
                                 "Setting netem latency and loss parameters failed on ifb0 interface")
