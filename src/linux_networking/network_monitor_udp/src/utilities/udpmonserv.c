#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "helpers.h"
#include "udpmon.h"

struct udpmon_pkt buff;
char magic[] = UDPMON_MAGIC;

int main(int argc, char **argv)
{
  if (argc != 2) { fprintf(stderr, "usage: udpmonserv <port_to_listen_on>\n"); return 1; }

  int udp_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in port;
  int len;

  if (udp_socket == -1) { perror("socket"); return 1; }

  port.sin_family = AF_INET;
  port.sin_port = htons(atoi(argv[1]));
  port.sin_addr.s_addr = INADDR_ANY;

  if (bind(udp_socket, (struct sockaddr *) &port, sizeof(port))) { perror("bind"); return 1; }
  
  while (1) 
  { 
    struct sockaddr src_addr;
    socklen_t addrlen = sizeof(src_addr);

    if ((len = recvfrom(udp_socket, &buff,  sizeof(buff), MSG_WAITALL, &src_addr, &addrlen)) == -1) { perror("recv"); return 1; }

    if (*((int *) &magic) == (*((int *) &buff.magic)))
      buff.echoed = gettime();
    else 
      continue;
      // Assume this is the old version
    //  ((struct udpmon_pkt_old *) &buff)->echoed = gettime();

    sendto(udp_socket, &buff, len, 0, &src_addr, addrlen);
  }

  return 0;
}
