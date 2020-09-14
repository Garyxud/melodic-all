### Resource request description
#
#   This tracks the progress of a single requested resource group.
#   All these resources will be granted, preempted or canceled
#   together as a unit.

##############################################################################
# Resource Identification
##############################################################################

uuid_msgs/UniqueID id   # Requester-assigned universally unique identifier
Resource[] resources    # List of requested rapps and platforms

##############################################################################
# State
##############################################################################

uint8 status            # Current status of this request
uint8 reason            # Reason for this status
string problem          # Convenience message for introspecting on the reason 

#  Status value labels:
uint8 NEW         = 0   # New request for the scheduler
uint8 RESERVED    = 1   # Request for a reservation at some future time
uint8 WAITING     = 2   # Request has been queued by the scheduler
uint8 GRANTED     = 3   # Request was granted by the scheduler
uint8 PREEMPTING  = 4   # The scheduler wants to preempt this
                        #   previously-granted request, but the
                        #   requester has not yet canceled it
uint8 CANCELING   = 5   # The requester wishes to cancel this
                        #   request, but the scheduler has not yet
                        #   confirmed that it is closed
uint8 CLOSED      = 6   # Request is now closed (terminal state)

# Reason labels:
uint8 NONE        = 0   # No reason provided
uint8 PREEMPTED   = 1   # Preempted for higher-priority task
uint8 BUSY        = 2   # Requested resource busy elsewhere
uint8 UNAVAILABLE = 3   # Requested resource not available
uint8 TIMEOUT     = 4   # Lost contact with requester
uint8 INVALID     = 5   # Ill-formed request: see problem string for details

##############################################################################
# Scheduling Variables
##############################################################################

time     availability   # Estimated time of availability (zero if unknown)
duration hold_time      # Estimated hold time once allocated (zero if unknown)
int16    priority       # Current priority of this request

# Priority labels:
int16 BACKGROUND_PRIORITY = -20000      # When nothing else to do
int16 LOW_PRIORITY = -10000             # Low-priority task
int16 DEFAULT_PRIORITY = 0              # Sane default priority
int16 HIGH_PRIORITY = 10000             # High-priority task
int16 CRITICAL_PRIORITY = 20000         # Mission-critical task
