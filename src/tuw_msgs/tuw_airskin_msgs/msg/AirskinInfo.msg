# This expresses a set of AirSkin pad pressures
Header header                  # timestamp in the header is the acquisition time and frame

uint8[] ids                    # I2C ids of target AirSkin pads
string[] names                 # names and TF frames names of corresponding AirSkin pads
uint32[] min                   # pressure min value
uint32[] max                   # pressure max value
