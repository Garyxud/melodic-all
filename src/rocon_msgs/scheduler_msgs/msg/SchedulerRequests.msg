### Scheduler Requests
#
#   Rocon services send this message on the scheduler's input topic to
#   make or update their resource requests.  All resources they
#   currently have or desire should be included.
#
#   If not all of the requested resources are immediately available,
#   the scheduler will queue those requests and provide feedback using
#   a message of the same type.  The requester should wait until
#   feedback indicates the associated resources are available.  It may
#   cancel some requests at any time via an new allocation message.
#
Header header                   # Time of this status, frame_id irrelevant
uuid_msgs/UniqueID requester    # Requester identifier
Request[] requests              # Status of every currently-known request
