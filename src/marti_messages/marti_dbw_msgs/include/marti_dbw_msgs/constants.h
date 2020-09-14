#ifndef MARTI_DBW_MSGS_DBW_CONSTANTS_H_
#define MARTI_DBW_MSGS_DBW_CONSTANTS_H_

namespace marti_dbw_msgs
{
// This file includes constants used in the marti_dbw_msgs ROS
// messages.
//
// This approach goes against the ROS convention of defining constants
// in messages (Although there is precendent for this approach.  e.g,
// sensor_msgs/image_encodings.h).  By keeping constants in a separate
// header file, we can add new constants without changing the identity
// of the message (ie, the md5sum).  This will hopefully reduce
// development friction overall by allowing us to easily add new
// states as necessary without having to convert old bag files.

//////////////////////////////////////////////////////////////
// Transmission ranges

// The unknown range is only valid as feedback. It should not be sent as a control.
const std::string TRANS_UNKNOWN = "unknown";

const std::string TRANS_PARK = "park";
const std::string TRANS_NEUTRAL = "neutral";
const std::string TRANS_REVERSE = "reverse";
const std::string TRANS_DRIVE_LOW = "drive_low";
const std::string TRANS_DRIVE_HIGH = "drive_high";
const std::string TRANS_PIVOT = "pivot";



//////////////////////////////////////////////////////////////
// Turn signal states

const std::string TURN_SIGNAL_UNKNOWN = "unknown";
const std::string TURN_SIGNAL_OFF = "off";
const std::string TURN_SIGNAL_LEFT = "left";
const std::string TURN_SIGNAL_RIGHT = "right";
const std::string TURN_SIGNAL_HAZARDS = "hazards";
}
#endif  // MARTI_DBW_MSGS_DBW_CONSTANTS_H_
