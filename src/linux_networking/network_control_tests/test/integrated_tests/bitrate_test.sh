#!/bin/bash

PACKET_SIZES=(150 1500 3000 6000 12000 24000 48000)

for PKTSIZE in ${PACKET_SIZES[@]}; do
  rosrun network_monitor_udp bitrate_test.py $PKTSIZE > bitrate_result_$PKTSIZE.plot
done
