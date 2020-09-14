#!/bin/sh

# run.sh : 全てのメモリーリークチェックスクリプトを実行する

# Naming service start check
nspid=`ps -af | grep '/omniNames -start 9898' | awk '{print $1}'`
if test "$nspid" != "" ; then
  pkill -f 'omniNames -start 9898'
  sleep 10
fi

# Naming service start
./rtm-naming 9898
echo "<<< Naming service port:9898 start >>>"
sleep 15

# Memory Leak check start
./AddRemoveMemberSDOPackageTest.sh
./AddRemoveOrganizationSDOPackageTest.sh
./AddRemoveRTCTest.sh
./AttachDetachRTCTest.sh
./ConfigurationSDOPackageTest.sh
./ConnectRTCTest.sh
./CreateDeleteRTCTest.sh
./DataPortTest.sh
./ManagerTest.sh
./OrganizationSDOPackageTest.sh
./RTCTest.sh
./SDOPackageTest.sh
./ServiceProfileSDOPackageTest.sh
./SetMemberSDOPackageTest.sh

# Naming service stop
sleep 5
pkill -f 'omniNames -start 9898'
echo "<<< Naming service stop >>>"

