# generated from rocon_launch/env-hooks/15.rocon_launch.zsh.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/shells/env.zsh"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "${CATKIN_ENV_HOOK_WORKSPACE}/share/rocon_launch/shells/env.zsh"
@[end if]@
