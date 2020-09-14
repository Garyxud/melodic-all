# Some commonly used constant strings in the concert.
#
# Note: avoid using hardcoded topic_name strings here. Prefer rostopic.find_by_type
# and rosservice.rosservice_find for getting services that should be unique
# (e.g. gateway/interactions). Refer to rocon_python_comms for some wrappers around these.

# Parameters
string PARAM_ROCON_SCREEN = /rocon/screen

# Scheduler
string SCHEDULER_UNALLOCATED_RESOURCE = unallocated

# Conductor
string CONCERT_CLIENTS = /concert/conductor/concert_clients

# Namespaces
string SERVICE_NAMESPACE = /services
string SOFTWARE_NAMESPACE = /software
