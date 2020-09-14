#ifndef DCCOMMS_ROS_MSGS_TYPES_H_
#define DCCOMMS_ROS_MSGS_TYPES_H_

#include <string>

namespace dccomms_ros {

using namespace std;

enum CHANNEL_LINK_TYPE { CHANNEL_TXRX = 0, CHANNEL_TX, CHANNEL_RX };
enum DEV_TYPE { ACOUSTIC_UNDERWATER_DEV, CUSTOM_DEV };
enum CHANNEL_TYPE { ACOUSTIC_UNDERWATER_CHANNEL, CUSTOM_CHANNEL };

static string DevType2String(DEV_TYPE dev) {
  string res;
  switch (dev) {
  case ACOUSTIC_UNDERWATER_DEV:
    res = "ACOUSTIC_UNDERWATER";
    break;
  case CUSTOM_DEV:
    res = "CUSTOM";
    break;
  }
  return res;
}

static string ChannelType2String(CHANNEL_TYPE v) {
  string res;
  switch (v) {
  case ACOUSTIC_UNDERWATER_CHANNEL:
    res = "ACOUSTIC_UNDERWATER";
    break;
  case CUSTOM_CHANNEL:
    res = "CUSTOM";
    break;
  }
  return res;
}
}

#endif
