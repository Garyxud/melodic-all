# generated from rocon_python_comms/env-hooks/15.rospair.zsh.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/completion/rospair.zsh"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "${CATKIN_ENV_HOOK_WORKSPACE}/share/rocon_python_comms/completion/rospair.zsh"
@[end if]@
