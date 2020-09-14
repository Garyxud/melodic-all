#! /bin/bash
while true; do
  ifconfig $4 >/dev/null 2>&1 && exec udhcpc "$@"
  echo Interface $4 not found. Sleeping for 10 seconds...
  sleep 10;
done
