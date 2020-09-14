# This message file defines commonly used constants
# for services' return codes

#  Default success without additional information
int16 SUCCESS=0

# Reserved error codes (reserved range [-100,-1])
int16 INVALID_ARGUMENT=-1   # An invalid argument was passed with the service request
int16 INTERNAL_ERROR=-2     # Internal error
int16 SENSOR_TIMEOUT=-4     # Timeout while waiting for image data to fulfill a request
int16 IO_ERROR=-7           # IO error
int16 NOT_APPLICABLE=-8     # Request not applicable (e.g. in current state)
int16 INVALID_LICENSE=-9    # No valid license to fulfill this request
int16 CAPACITY_EXCEEDED=-10 # capacity exceeded, i.e. new element could not be added

# Reserved non-error codes (reserved range [1,100])
int16 CAPACITY_REACHED=10   # max capacity is reached, i.e. list is now full

# each module defines own msg-file with consts for the module specific return codes
