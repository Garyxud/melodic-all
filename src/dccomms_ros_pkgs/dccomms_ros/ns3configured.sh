#!/bin/bash

cd modules/ns-3-dev/
wafout=$(./waf --check-profile)

if [[ $wafout =~ "Build profile" ]]
then
	echo "true"
else
	echo "false"
fi
cd -

