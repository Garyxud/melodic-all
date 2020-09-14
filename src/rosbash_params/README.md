# rosbash_params

[![Build Status](http://build.ros.org/buildStatus/icon?job=Idoc__rosbash_params__ubuntu_trusty_amd64)](http://build.ros.org/job/Idoc__rosbash_params__ubuntu_trusty_amd64/)

This Bash env-hook adds a "node-like" interface to your code written in Bash.
The main thing it adds is ROS-like command-line parameter parsing (`_param:=value`), so that you can easily call the 
Bash script from a launch file like 
`<node name="test" pkg="pkg" type="my_bash_script.sh"><param name="par" value="test" /></node>`.

## Advantages

* Adds named parameters for Bash scripts.
    * `_param:=value`
* Position of the parameters doesn't matter (though you can still easily pass positional arguments).
    * `_param:=value positional_arg1 positional_arg2 _andrew:=martin`
        * `positional_arg1` and `positional_arg2` are accessible to the script as positional args as if there were 
        no `:=` param mappings.
* Super-easy parameter parsing.
    * `rosbash_param var "param" "default"`.
* Unified representation of bool values:
    * `true`, `True`, `yes`, `on` and `1` all translated to a single value `True`
    * `false`, `false`, `no`, `off` and `0` are all translated to a single value `False`
    * Notice: talking about exit-codes, `0` usually means success, and non-`0` means failure. The unified bool 
    representation works with the opposite meanings. So pay attention when setting bool parameters from exit-codes.
* Can also be used standalone outside ROS pacakges.
    * You need just `rospy` ROS package. You don't need a ROS master (`roscore`) running if you don't need to access 
    ROS param server.
    
## Usage

### Example script test_rosbash

    #!/usr/bin/env bash
    
    rosbash_init_node "node_name" "$@"  # parse the command line arguments
    
    rosbash_param mandatory_param "param_name"  # if default value is not specified, the param is mandatory
    rosbash_param optional_param "param2_name" "default_value" # optional param
    rosbash_param bool_param1 "bool_name1" # bool param without default
    rosbash_param bool_param2 "bool_name2" "True" # bool param with default
    rosbash_param bool_param3 "bool_name3" "False" # bool param with default
    
    echo "mandatory_param = ${mandatory_param}"  # access the parsed parameter value
    echo "optional_param = ${optional_param}"  # access the parsed parameter value
    echo "bool_param1 = ${bool_param1}"  # access the parsed parameter value
    echo "bool_param2 = ${bool_param2}"  # access the parsed parameter value
    echo "bool_param3 = ${bool_param3}"  # access the parsed parameter value
    
    echo "rosbash_unused_argv = ${rosbash_unused_argv[@]}"  # all CLI args not parsed as a parameter
    
### Example call:

    $ ./test_rosbash _param_name:=1 _unparsed_param:=2 positional1 positional2 _bool_name1:=False
    mandatory_param = 1
    optional_param = default_value
    bool_param1 = False
    bool_param2 = True
    bool_param3 = False
    rosbash_unused_argv = _unparsed_param:=2 positional1 positional2

### Example call with missing mandatory parameter:

    $ ./test_rosbash positional1 positional2
    Required parameter 'param_name' was not set.
    
### Example call showing bool behavior

    $ ./test_rosbash  _param_name:=test _bool_name1:=1 _bool_name2:=0 _bool_name3:=on
    mandatory_param = test
    optional_param = default_value
    bool_param1 = 1  # without default value, we cannot safely convert all `1`s to `True`
    bool_param2 = False  # with default value either `True` or `False`, we can convert `1` to `True` and `0` to `False`
    bool_param3 = True  # `on` without quotes is always converted to `True`
    rosbash_unused_argv = 
    
### Example launch file

    <launch>
        <node name="test" pkg="test_pkg" type="test_rosbash">
            <param name="param_name" value="test" />
            <param name="param2_name" value="optional" />
            <param name="bool_name1" value="off" />
        </node>
    </launch>
    
## API

### rosbash_init_node

#### Arguments

* `rosbash_node_name` Name of the "node". If `__name:=name` param mapping is present, it overrides this value. The 
node name specifies prefix of the parameters on the param server.
* all other arguments are to be parsed as parameters (call with `"$@"` to pass all script args)

#### Global variables set by this function

* `rosbash_unused_params`: associative array of parsable params on CLI that were not used by any call to 
`rosbash_param`. Keys are parameter names, values are their values.
* `rosbash_unused_argv`: all arguments to this function from which no parameter was parsed (as Bash array; use `arg_string="${rosbash_unused_argv[@]}"` to convert to space-delimited string).
* `_rosbash_params`: do not use, is private

### rosbash_param

**Be sure to call `rosbash_init_node` before calling this function!**

#### Arguments

* `result_var` The variable to store the result in (pass without dollar sign).
* `param` Name of the parameter to read.
* `default` [optional] The value to use when the parameter is not set. If no default is set, a missing parameter
results in calling `exit 1` or `return 1` (depending on the value of `ROS_BASH_PARAM_EXIT`). To correctly convert 
`0` and `1` to `False` and `True`, you have to specify a default value `True` or `False` to bool params. If you don't, 
`0` and `1` will not be converted to their logical values.
                   
#### Environment variables

* `ROS_BASH_USE_PARAM_SERVER` (default true): Whether to look for parameter values to the ROS param server. Also 
automatically store the parsed param values to the parameter server. Set to `"0"` to disable.
* `ROS_BASH_USE_PARAM_VERBOSE` (default false): Set to any nonempty string but `"0"` to enable verbose logging.
* `ROS_BASH_PARAM_EXIT` (default true): If set to `"0"`, do not call `exit 1` when parameter is not found, instead
call `return 1`, so that the calling code controls what should happen when the parameter is not found.
    * e.g. `ROS_BASH_PARAM_EXIT=0 rosbash_param var "mandatory" || echo "Please, fill the mandatory param"`
    
## Other shells

As I don't use any other shells, this package only supports Bash. But theoretically it can work in many other shells,
so if you want them supported, feel free to send a pull request (not issues, I won't write support for other shells 
myself).
