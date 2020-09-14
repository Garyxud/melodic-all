#!@RCD_SCRIPTS_SHELL@
#
# @file omninames.sh
# @brief omniORB NameService rc.d control script
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# PROVIDE: omninames
# REQUIRE: DAEMON
# KEYWORD: shutdown
#
# Some variables in /etc/rc.d/rc.conf should be set for omniNames
#
# In order to start omniNames, set YES ([yY][eE][sS] is allowed)
# to a variable "omniNames" as follows.
#
# omniNames=YES
#
# omniNames listens on 2809 port as default. To listen on a different port
# set a variable omniNames_port as follows.
#
# omniNames_port="9876"	
#
# omniNames's log/data directory for log and persistent object reference 
# can be set by omniNames_logdir.
#
# omniNames_logdir="/var/omninames" 
#
# Error log file name can be set as follows.
#
# omniNames_errlog="/var/log/omniNames.log" 
#
# Other omniNames options can be set by "omniNames_flags". This option 
# override other options. See omniNames(1) for possible options.
#
# omniNames_flags="-start 567 -logdir /tmp/omniorb/" 
#

# global variables
NS="omniNames"
COMMAND="@PREFIX@/bin/${NS}"
NS_PORT="2809"
NS_LOGDIR="/var/omninames"
NS_LOGFILE="omninames-"`hostname`.log
NS_ERRLOG="/var/log/omniNames.log"
NS_OPTIONS=""
PID_FILE="/var/run/omninames.pid"


. /etc/rc.d/rc.conf


# checking omniName="yes/no" in rc.local
if test -z $omniNames; then
    exit 0
fi
ENABLE_NS=`echo $omniNames | sed -e 's/[yY][eE][sS]/yes/g'`
if test ! "x$ENABLE_NS" = "xyes"; then
    exit 0
fi


#------------------------------
# functions
#------------------------------
usage()
{
    echo "Usage: omninames (start|stop)"
    echo ""
}

create_logdir()
{
    if ! test -d $NS_LOGDIR; then
        echo "$NS_LOGDIR not found!"
        mkdir -p $NS_LOGDIR
    fi
}

create_options()
{
    # setting port
    local port=$omniNames_port
    if test ! -z $port; then
        NS_PORT=$port
    fi
    NS_OPTIONS="$NS_OPTIONS -start $NS_PORT -always"

    # setting logdir
    local logdir=$omniNames_logdir
    if test ! -z $logdir; then
        if test ! -d $logdir; then
            mkdir -p $logdir
        fi
        NS_LOGDIR=$logdir
    fi
    NS_OPTIONS="$NS_OPTIONS -logdir $NS_LOGDIR"

    # setting errlog
    local errlog=$omniNames_errlog
    if test ! -z $errlog; then
        NS_ERRLOG=$errlog
    fi
    NS_OPTIONS="$NS_OPTIONS -errlog $NS_ERRLOG"

    # overwrite omniNames options
    local options=$omniNames_flags
    if test ! -z $options; then
        NS_OPTIONS="$options"
    fi
}

start_omninames()
{
    create_logdir
    create_options
    echo "Starting $NS"
    $COMMAND $NS_OPTIONS &
    if test ! $? -eq 0; then
        echo "Starting $NS failed."
        exit 1
    else
        local pid=$!
        echo $pid > $PID_FILE
    fi
}

stop_omninames()
{
    if test -f $PID_FILE; then
        echo "Stopping $NS, PID:" `cat $PID_FILE`
        kill `cat $PID_FILE`
        killok=$?
        rm -f $PID_FILE
        if test $killok -eq 0; then
            return 0
        fi
    fi
    local pid=`ps -ef | grep 'omniNames' | grep -v 'grep' | sort -u | awk '{print $2;}'`
    if test -z $pid; then
        echo "omniNames does not exist."
        return 1
    fi
    echo "Stopping $NS"
    kill $pid
}


#--------------------
# main
#--------------------
if test $# -gt 1; then
    usage
    exit 1
fi

if test $# -eq 0; then
    option="start"
else
    option=$1
fi
case $option in
    start)
        start_omninames
        ;;
    stop)
        stop_omninames
        ;;
    *)
        echo "Unknown option:"$option
        usage
        exit 1
        ;;
esac

