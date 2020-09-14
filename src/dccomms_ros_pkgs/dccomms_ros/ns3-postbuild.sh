#!/bin/bash

filename=$(basename $1)

dstfile=$2/$filename
if [ -f $dstfile ]
then
	echo "ns3 lib already exists: $dstfile" | tee -a ns3-postbuild.log
	cp $1 $2
else
	echo "copying \"$1\" into $2..." | tee -a ns3-postbuild.log
	cp $1 $2
fi
echo "ok"
