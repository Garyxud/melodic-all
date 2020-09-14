# This cmake function parses a list of shortened (develspace) filenames in the form of:
# msg/GoalID.msg;msg/GoalStatus.msg;msg/GoalStatusArray.msg
#
# or absolute paths (installspace):
# /opt/ros/indigo/share/msg/GoalID.msg;/opt/ros/indigo/share/msg/GoalStatus.msg;
#
# and puts the message names into a canonical form for easier processing
# actionlib_msgs/GoalID;actionlib_msgs/GoalStatus

function(get_base_names FILELIST TYPE PACKAGENAME OUTPUT_NAMES)
  if (${FILELIST})
    string(REGEX REPLACE "(\\/|${TYPE})[a-z0-9_\\/.~-]+"
      "${PACKAGENAME}/" OUTPUT_VAR
      ${${FILELIST}}
    )
    string(REGEX REPLACE "\\.(${TYPE})*"
      ";" OUTPUT_VAR
      ${OUTPUT_VAR}
    )
    set(${OUTPUT_NAMES} ${OUTPUT_VAR} PARENT_SCOPE)
  elseif()
    set(${OUTPUT_NAMES} "" PARENT_SCOPE)
  endif()
endfunction()
