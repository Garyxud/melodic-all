### Known resources.
#
#   A ROCON scheduler uses this message to report the status of all
#   the resources it knows about and manages.
#
Header header                   # Time of this status, frame_id irrelevant
CurrentStatus[] resources       # Status of every currently-known resource
