#!/bin/sh

# 各種オペレーションのメモリーリークチェックスクリプトを実行する

# Naming service start check
nspid=`ps -af | grep '/omniNames -start 9898' | awk '{print $1}'`
if test "$nspid" = "" ; then
  ./rtm-naming 9898
  echo "<<< Naming service port:9898 start >>>"
  sleep 15
fi

# rtcd start check
rtcdpid=`ps -af | grep '/.libs/lt-rtcd' | awk '{print $1}'`
if test "$rtcdpid" != "" ; then
  pkill -f '/.libs/lt-rtcd'
  sleep 10
fi

# rtcd start
./rtcd &
echo "<<< rtcd start >>>"
sleep 10

# Memory Leak check start
echo "<<< Memory leak check start >>>"
./ConnectRTCTest.py
sleep 2

# rtcd stop
pkill lt-rtcd
sleep 5
echo "<<< rtcd stop >>>"
echo ""

