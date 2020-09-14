#!/bin/sh
#
# @file run.sh
# @brief SimpleIO example startup script
# @date $Date: 2007-04-27 06:12:58 $
#
# Copyright (c) 2003-2007 Noriaki Ando <n-ando@aist.go.jp>
#          Task-intelligence Research Group,
#          Intelligent System Research Institute,
#          National Institute of Industrial Science (AIST), Japan
#          All rights reserved.
#
# $Id$
#

nsport='9876'
hostname=`hostname`


term=`which kterm`

if test "x$term" = "x" ; then
    term=`which xterm`
fi

if test "x$term" = "x" ; then
    term=`which uxterm`
fi

if test "x$term" = "x" ; then
    term=`which gnome-terminal`
fi

if test "x$term" = "x" ; then
    echo "No terminal program (kterm/xterm/gnome-terminal) exists."
    exit
fi

../../utils/rtm-naming/rtm-naming $nsport

echo 'corba.nameservers: 127.0.0.1:'$nsport > ./rtc.conf
echo 'naming.formats: %n.rtc' >> ./rtc.conf
echo 'logger.log_level: TRACE' >> ./rtc.conf

$term -e ./ConsoleInComp &
$term -e ./ConsoleOutComp &

sleep 5
./ConnectorComp --port $nsport --flush &

#sleep 10
#nspid=`ps -ax | grep 9876 | awk '{print $1}'`
#kill $nspid
#echo 'Naming service was stopped.'
