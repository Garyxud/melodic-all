std_msgs/Header header

# Constants for commonly used units
string UNIT_SEC=sec
string UNIT_MILLI_SEC=msec
string UNIT_MICRO_SEC=usec
string UNIT_PERCENTAGE=percent
string UNIT_COUNT=count
string UNIT_COUNT_PER_SEC=count_per_sec
string UNIT_BYTES=bytes
string UNIT_KILOBYTES=kilobytes
string UNIT_MEGABYTES=megabytes
string UNIT_GIGABYTES=gigabytes
string UNIT_TERABYTES=terabytes
string UNIT_BYTES_PER_SEC=bytes_per_sec
string UNIT_KILOBYTES_PER_SEC=kilobytes_per_sec
string UNIT_MEGABYTES_PER_SEC=megabytes_per_sec
string UNIT_GIGABYTES_PER_SEC=gigabytes_per_sec
string UNIT_TERABYTES_PER_SEC=terabytes_per_sec
string UNIT_NONE=none

# Constant for NA string
string NA_STRING=non_applicable_string_value

#####################################
# required fields

# The name for the metric
string metric_name

# The units applied to the value
string unit

# The value of the metric
float64 value

# The time since the epoch
time time_stamp

# optional Custom dimensions used to group/sort metrics
MetricDimension[] dimensions
