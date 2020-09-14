#ifndef __UDPMON_H__
#define __UDPMON_H__

// First magic character is 0xEF, a convenient opcode for wg-openvpn.
#define UDPMON_MAGIC { (unsigned char) 0xEF, (unsigned char) 0x41, (unsigned char) 0xc6, (unsigned char) 0x35 }
#define UDPMON_MAGIC_ROS { 0xEF, 0x41, 0xc6, 0x34 }

struct __attribute__((__packed__)) udpmon_pkt
{
  char magic[4];
  double sent;
  double echoed;
  int seqnum;
  int source_id;
  char padding[4096];
};

struct __attribute__((__packed__)) udpmon_pkt_old
{
  double sent;
  double echoed;
  int seqnum;
  char padding[4096];
};

#endif
