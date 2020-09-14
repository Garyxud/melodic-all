# Used by the remocons to inform the concert of it's current status. They should
# publish this as a latched publisher.

rocon_std_msgs/MasterInfo platform_info
# The remocon id
# This should be a '32 character Type 4 uuid hex string'
string uuid


# We should be using this, but java can't handle the type, reintegrate when we bugfix.
# uuid_msgs/UniqueID[] running_interactions

# This is a crc32 hash code we use because of the above.
int32[] running_interactions

# rocon version compatibility identifier (used when connecting to concerts)
string version
