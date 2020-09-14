#! /bin/sh

#if [ $1 = bound ]; then
#  ifconfig $interface $ip netmask $subnet
#  ip route replace default via $router dev $interface # FIXME, this could be a list.
#fi

echo UDHCPC: $1 $interface $ip $subnet $router
