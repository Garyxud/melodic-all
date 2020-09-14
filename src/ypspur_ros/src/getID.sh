#!/bin/bash

tmpfile=$(mktemp)

for i in {1..10};
do
	if [ ! -e $1 ]; then
		sleep 0.5;
		continue;
	fi
	sleep 0.5
	stty -F $1 raw -echo 2> /dev/null

	if [ ! $? -eq 0 ]; then
		continue;
	fi

	sleep 0.2
	cat < $1 > $tmpfile 2> /dev/null &
	catpid=$!

	echo -e "QT\nVV\n" > $1
	sync
	sleep 0.5

	kill -SIGTERM "$catpid"

	grep -e "^SERI:\([0-9a-zA-Z]*\);.*$" $tmpfile 2> /dev/null > /dev/null
	if [ $? -eq 0 ]; then
		break;
	fi
done 2> /dev/null

grep -e "^SERI:\([0-9a-zA-Z]*\);.*$" $tmpfile 2> /dev/null | sed -e "s/^SERI:\([0-9a-zA-Z]*\);.*$/\1/" | tail -n1

rm $tmpfile

