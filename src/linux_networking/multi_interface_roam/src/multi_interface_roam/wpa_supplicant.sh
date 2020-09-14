#! /bin/bash
BASE=`rospack find wpa_supplicant`
exec $BASE/wpa_supplicant/wpa_supplicant/wpa_supplicant "$@"
