#! /bin/sh
# Clean up previous run

if [ $# -ne 6 ]; then
  echo "Usage: hwsim_nat_setup.sh <IF1> <REAL_IP1> <FAKE_IP1> <IF2> <REAL_IP2> <FAKE_IP2>"
  echo "Example: hwsim_nat_setup.sh wlan0 192.168.68.1 192.168.69.1 wlan1 192.168.69.2 192.168.68.2"
  echo
  echo "Note that FAKE_IP1 has to be in the same subnet as REAL_IP2 (and FAKE_IP2 in same subnet as REAL_IP1)"
  exit 1
fi

IF1=$1
REAL_IP1=$2
FAKE_IP1=$3

IF2=$4
REAL_IP2=$5
FAKE_IP2=$6
 
sudo iptables -F -t nat

# Set up current run

sudo ifconfig $IF2 up
sudo ifconfig $IF1 up
sudo ifconfig $IF1 $REAL_IP1 netmask 255.255.255.0
sudo ifconfig $IF2 $REAL_IP2 netmask 255.255.255.0
sudo arp -Ds $FAKE_IP2 $IF2
sudo arp -Ds $FAKE_IP1 $IF1
sudo iptables -A POSTROUTING -t nat -s $REAL_IP1 -o $IF1 -j SNAT --to-source $FAKE_IP1
sudo iptables -A PREROUTING -t nat -d $FAKE_IP2 -i $IF2 -j DNAT --to-dest $REAL_IP2
sudo iptables -A POSTROUTING -t nat -s $REAL_IP2 -o $IF2 -j SNAT --to-source $FAKE_IP2
sudo iptables -A PREROUTING -t nat -d $FAKE_IP1 -i $IF1 -j DNAT --to-dest $REAL_IP1

# Make sure we don't have any connections that don't use the new iptables rules.
sudo conntrack -F 
