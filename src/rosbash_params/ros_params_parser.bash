#---
## @brief ROS CLI params parser for bash 4.2+
## @see See readme for usage.
#---

#---
## @Brief Initialize ROS param mappings parser, and establish a node-like namespace for the parameter server.
##
## @param rosbash_node_name Name of the "node". If '__name:=name' param mapping is present, it overrides this value.
## @param argv All other arguments are used for parameter parsing, are passed to `rosbash_unused_argv` etc.
## @globals rosbash_unused_params, rosbash_unused_argv, _rosbash_params (private)
#---
function rosbash_init_node
{
    rosbash_node_name="$1"
    shift

    local script="$(cat <<'SCRIPT'
import sys
from rospy.client import load_command_line_node_params
params=load_command_line_node_params(sys.argv)
str_params=["[\"%s\"]=\"%s\"" % (k,v) for (k,v) in params.iteritems()]
print(" ".join(str_params))
SCRIPT
)"

    local params_str="$(python -c "${script}" "$@")"
    declare -gA _rosbash_params="( ${params_str} )"
    declare -gA rosbash_unused_params="( ${params_str} )"

    declare -ga rosbash_unused_argv=()
    for arg in "$@"; do
        # support assigning node name via __name:=name; load_command_line_node_params ignores params starting with __
        if [[ "${arg}" == __name:=* ]]; then
            rosbash_node_name="${arg#__name:=}"
        else
            rosbash_unused_argv+=("${arg}")
        fi
    done
}
export -f rosbash_init_node

#---
## @brief Read a ROS param $param value and store it to $result_var.
##
## @param result_var The variable to store the result in.
## @param param Name of the parameter to read.
## @param default [optional] The value to use when the parameter is not set. If no default is set, a missing parameter
##                           results in calling 'exit 1'.
##
## @env ROS_BASH_USE_PARAM_SERVER (default true) Whether to look for parameter values to the ROS param server.
##                                               Set to "0" to disable.
## @env ROS_BASH_USE_PARAM_VERBOSE (default false) Set to any nonempty string but "0" to enable verbose logging.
## @env ROS_BASH_PARAM_EXIT (default true) If set to "0", do not call `exit 1` when parameter is not found, instead
##                                         call `return 1`, so that the calling code controls what should happen when
##                                         the parameter is not found.
##
## @note Requires calling <@function rosbash_init_node> before using this one.
#---
function rosbash_param
{
    local result_var="$1"
    local param="$2"
    local default="$3"

    local default_set=false; [[ $# -gt 2 ]] && default_set=true
    local use_param_server=true; [[ "${ROS_BASH_USE_PARAM_SERVER}" == "0" ]] && use_param_server=false
    local verbose=false; [[ "${ROS_BASH_PARAM_VERBOSE}" ]] && [[ "${ROS_BASH_PARAM_VERBOSE}" != "0" ]] && verbose=true
    local exit_when_not_found=true; [[ "${ROS_BASH_PARAM_EXIT}" == "0" ]] && exit_when_not_found=false

    # search for the parameter value

    local value_found=false
    # the parameter was set on command line
    if [[ ${_rosbash_params["${param}"]+_} ]]; then
        if ${verbose}; then echo "Using command line value '${_rosbash_params["${param}"]}' for param ${param}"; fi
        eval ${result_var}="'${_rosbash_params["${param}"]}'"
        value_found=true
    fi

    # the parameter was not set on command line, so try searching on the param server
    if ! ${value_found} && ${use_param_server}; then
        if ${verbose}; then echo -n "Searching for ${param} on param server as ${rosbash_node_name}/${param} ... "; fi
        # local declaration has to be separate when used in conjuntion with command substitution:
        # https://stackoverflow.com/questions/4421257/why-does-local-sweep-the-return-code-of-a-command
        local value
        value=$(rosparam get "${rosbash_node_name}/${param}" 2>/dev/null)
        if [[ "$?" == "0" ]]; then
            eval ${result_var}="'$value'"
            value_found=true
            if ${verbose}; then echo "found"; fi
        else
            if ${verbose}; then echo "not found"; fi
        fi
    fi

    # the parameter was neither set, nor found on param server, but a default is specified
    if ! ${value_found} && ${default_set}; then
        if ${verbose}; then echo "Using default value '${default}' for param ${param}"; fi
        eval ${result_var}="'${default}'"
        value_found=true
    fi

    # parameter value was not found and no default is set
    if ! ${value_found}; then
        echo "Required parameter ${param} was not set." 1>&2
        if ${exit_when_not_found}; then
            exit 1
        else
            return 1
        fi
    fi

    # if default is set to True or False, we know it is a bool, and can thus also convert 0 and 1 to False and True
    if ${default_set}; then
        if [[ "${default}" == "True" || "${default}" == "False" ]]; then
            if [[ ${!result_var} == "1" ]]; then
                eval ${result_var}="True"
            elif [[ ${!result_var} == "0" ]]; then
                eval ${result_var}="False"
            fi
        fi
    fi

    if ${verbose}; then echo "Loaded parameter '${param}' with value '${!result_var}'"; fi

    if ${use_param_server}; then
        rosparam set "${rosbash_node_name}/${param}" "${!result_var}"
    fi

    unset rosbash_unused_params["${param}"]

    local param_escaped=$(LC_ALL=C sed 's/([^^])/[&]/g; s/\^/\\^/g' <<<"${param}")
    if [[ "${!result_var}" == "True" ]]; then
        local value_escaped="[a-zA-Z1-9]\+[0-9]*"  # might have been true, on, yes, 1, 2 etc.
    elif [[ "${!result_var}" == "False" ]]; then
        local value_escaped="[a-zA-Z0]\+"  # might have been false, off, no, 0 etc.
    else
        local value_escaped=$(LC_ALL=C sed 's/[^^]/[&]/g; s/\^/\\^/g' <<<"${!result_var}")
    fi

    local unset_regex="_${param_escaped}:=[\"']\?${value_escaped}[\"']\? \?"
    for i in "${!rosbash_unused_argv[@]}"; do
      $(echo "${rosbash_unused_argv[$i]}" | grep -q "${unset_regex}") && unset -v "rosbash_unused_argv[$i]"
    done
}
export -f rosbash_param
