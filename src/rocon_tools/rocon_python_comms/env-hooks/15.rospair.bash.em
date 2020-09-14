# generated from rocon_python_comms/env-hooks/15.rospair.bash.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/completion/rospair.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "${CATKIN_ENV_HOOK_WORKSPACE}/share/rocon_python_comms/completion/rospair.bash"
@[end if]@
