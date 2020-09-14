### ROCON resource current status.
#
#   A ROCON scheduler uses this message to report the status of each
#   resource it manages.
#

##############################################################################
# Resource Identification
##############################################################################

#   A canonical ROCON Uniform Resource Identifier describing this
#   resource, a fully-resolved character string in the form:
#
#      rocon:/platform/name/framework/os
#
string uri

##############################################################################
# Status
##############################################################################

uint8 status            # Current status of this resource

#  Status value labels:
uint8 AVAILABLE   = 0   # Available for use
uint8 ALLOCATED   = 1   # Allocated to some ROCON request
uint8 MISSING     = 2   # Not currently responding
uint8 GONE        = 3   # Has left the concert (cleanly)

#   The owner is the unique identifier of the Request message to which
#   this resource is currently assigned, or a zero UUID if it is not
#   assigned.  An AVAILABLE resource never has an owner, but a MISSING
#   one might.
uuid_msgs/UniqueID owner

#   The priority of an ALLOCATED resource is the priority of its
#   owning request.  An AVAILABLE resource never has a non-zero
#   priority, but a MISSING one might.
int16 priority

#   List of ROCON application names currently available with this
#   resource. The name string is usually a ROS "package/node"
#   identifier, unique because ROS package names are unique.  The
#   contents of this list could change over time due to the device's
#   own activities.
string[] rapps
