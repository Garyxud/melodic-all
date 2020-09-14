# Possible behaviour states
uint8 INVALID = 1
uint8 RUNNING = 2
uint8 SUCCESS = 3
uint8 FAILURE = 4

# Possible types of behaviour
uint8 UNKNOWN_TYPE = 0
uint8 BEHAVIOUR = 1
uint8 SEQUENCE = 2
uint8 SELECTOR = 3
uint8 PARALLEL = 4
uint8 CHOOSER = 5
uint8 DECORATOR = 6

# Blackbox Level - these must match what is in py_trees.common.BlackBoxLevel
uint8 BLACKBOX_LEVEL_DETAIL = 1
uint8 BLACKBOX_LEVEL_COMPONENT = 2
uint8 BLACKBOX_LEVEL_BIG_PICTURE = 3
uint8 BLACKBOX_LEVEL_NOT_A_BLACKBOX = 4

# Name given to the behaviour on construction
string name

# Name of the class of this behaviour
string class_name

# The ID of this behaviour
uuid_msgs/UniqueID own_id

# The ID of the parent behaviour (what if it's empty?)
uuid_msgs/UniqueID parent_id

# ID of the tip of this behaviour. If the tip is the behaviour itself or None,
# this is empty
uuid_msgs/UniqueID tip_id

# The IDs of children of this behaviour, if any
uuid_msgs/UniqueID[] child_ids

# A composite, sequence, selector, plain behaviour, etc.
uint8 type

# Blackbox level (for visualisation purposes)
uint8 blackbox_level

# The current state of execution. Can be invalid, running, success or failure
uint8 status

# The message attached to the current execution state
string message

# Whether this node was part of the last tick
bool is_active
