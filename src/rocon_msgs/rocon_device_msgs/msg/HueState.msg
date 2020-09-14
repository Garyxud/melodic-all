#pre-define color
string OFF = "OFF"
string WHITE = "WHITE"
string RED = "RED"
string GREEN = "GREEN"
string BLUE = "BLUE"
string YELLOW = "YELLOW"
string ORANGE = "ORANGE"
string MAGENTA = "MAGENTA"
string VIOLET = "VIOLET"

#light on/off status true:on /false:off
bool on 

#color coordination in HSV color space 
#http://en.wikipedia.org/wiki/HSL_and_HSV
uint16 hue #h
uint8 sat #s
uint8 bri #v

#light valid status - valid:on /invalid:off
bool reachable  

#for set color with pre-define color
string color