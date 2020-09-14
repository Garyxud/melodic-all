function _roscomplete_rospair {
    local arg opts
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]]; then
        opts="echo list call type info"
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    elif [[ $COMP_CWORD -ge 2 ]]; then
        if [[ ${arg} =~ \-\-.* ]]; then
            case ${COMP_WORDS[1]} in
                call)
                    COMPREPLY=($(compgen -W "--rate --once --file --latch" -- ${arg}))
                    ;;
                echo)
                    COMPREPLY=($(compgen -W "--bag --filter --nostr --noarr --clear --all offset" -- ${arg}))
                    ;;
                list)
                    COMPREPLY=($(compgen -W "--bag --verbose --host" -- ${arg})) 
                    ;;
            esac
        else
            case ${COMP_WORDS[1]} in
                type|info)
                    if [[ ${COMP_WORDS[$(( $COMP_CWORD - 1 ))]} == "-b" ]]; then
                        COMPREPLY=($(compgen -f -- ${arg}))
                    else
                        opts=`rostopic list 2> /dev/null`
                        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
                    fi
                    ;;
                call)
                    if [[ $COMP_CWORD == 2 ]]; then
                        opts=`rospair list 2> /dev/null`
                        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
                    elif [[ $COMP_CWORD == 3 ]]; then
                        type=`rospair type ${COMP_WORDS[2]} | sed -e 's/Pair//g'`
                        opts=`rosmsg-proto msg 2> /dev/null -s ${type}Request`
                        if [ 0 -eq $? ]; then
                            COMPREPLY="$opts"
                        fi
                    fi
                ;;
            esac
        fi
    fi
}

complete -F "_roscomplete_rospair" "rospair"