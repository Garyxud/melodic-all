# This expresses a set of AirSkin pad colors
Header header                  # timestamp in the header is the acquisition time and frame

uint16[] idx                   # index of the color in the colors array 
                               # if empty the colors array must match the target system
std_msgs/ColorRGBA[] colors    # RGBA color values for corresponding AirSkin pads
