#!/bin/sh

chmod 644 *.h *.cpp
headers=`ls *.h`
cpps=`ls *.cpp`

for h in $headers; do
    cpp=`echo $h | sed 's/\.h$/\.cpp/'`
    if test \! -f $cpp ; then
	echo $h
    fi
done