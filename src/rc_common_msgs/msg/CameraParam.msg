# Camera parameters that relate to a published image

# Header with timestamp and frame_id to which the parameters belong
Header header

# True in case of a color camera
bool is_color_camera

# Exposure time of the camera in seconds
float32 exposure_time

# Gain factor in dB
float32 gain

# Line status for DigitalIO control according to GenICam SFNC:
# Current status of all available line signals in a single bitfield
uint32 line_status_all

# Line sources for DigitalIO control according to GenICam SFNC:
# Specifies for each line which internal acquisition or I/O source
# signal is output on the respective line.
#
# The keys define the lines, the values define the sources; e.g.
# [ {key: "out1", value: "ExposureActive"},
#   {key: "out2", value: "Low"},
#   ...
# ]
#
KeyValue[] line_source

# Additional optional camera paramters as key value pairs
KeyValue[] extra_data
