trap "exit" INT TERM ERR
trap "kill 0" EXIT

rosrun roscpp_tutorials talker > /dev/null &
rosrun roscpp_tutorials listener & 

wait
