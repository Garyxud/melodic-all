# generated from rosbash_params/env-hooks/30.rosbash_params.bash.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/ros_params_parser.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/rosbash_params/ros_params_parser.bash"
@[end if]@