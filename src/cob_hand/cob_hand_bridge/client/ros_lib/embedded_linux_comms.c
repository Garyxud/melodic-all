/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef ROS_EMBEDDED_LINUX_COMMS_H
#define ROS_EMBEDDED_LINUX_COMMS_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <assert.h>

#define DEFAULT_PORTNUM 11411

void error(const char *msg)
{
  perror(msg);
  exit(0);
}

void set_nonblock(int socket)
{
  int flags;
  flags = fcntl(socket, F_GETFL, 0);
  assert(flags != -1);
  fcntl(socket, F_SETFL, flags | O_NONBLOCK);
}

int elCommInit(const char *portName, int baud)
{
  struct termios options;
  int fd;
  int sockfd;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  int rv;

  if (*portName == '/')       // linux serial port names always begin with /dev
  {
    printf("Opening serial port %s\n", portName);

    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
      // Could not open the port.
      perror("init(): Unable to open serial port - ");
    }
    else
    {
      // Sets the read() function to return NOW and not wait for data to enter
      // buffer if there isn't anything there.
      fcntl(fd, F_SETFL, FNDELAY);

      // Configure port for 8N1 transmission, 57600 baud, SW flow control.
      tcgetattr(fd, &options);
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_cflag &= ~CRTSCTS;
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      options.c_iflag &= ~(IXON | IXOFF | IXANY);
      options.c_oflag &= ~OPOST;

      // Set the new options for the port "NOW"
      tcsetattr(fd, TCSANOW, &options);
    }
    return fd;
  }
  else
  {
    // Split connection string into IP address and port.
    const char* tcpPortNumString = strchr(portName, ':');
    long int tcpPortNum;
    char ip[16];
    if (!tcpPortNumString)
    {
      tcpPortNum = DEFAULT_PORTNUM;
      strncpy(ip, portName, 16);
    }
    else
    {
      tcpPortNum = strtol(tcpPortNumString + 1, NULL, 10);
      strncpy(ip, portName, tcpPortNumString - portName);
    }

    printf("Connecting to TCP server at %s:%ld....\n", ip, tcpPortNum);

    // Create the socket.
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
      error("ERROR opening socket");
      exit(-1);
    }

    // Disable the Nagle (TCP No Delay) algorithm.
    int flag = 1;
    rv = setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag));
    if (rv == -1)
    {
      printf("Couldn't setsockopt(TCP_NODELAY)\n");
      exit(-1);
    }

    // Connect to the server
    server = gethostbyname(ip);
    if (server == NULL)
    {
      fprintf(stderr, "ERROR, no such host\n");
      exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(tcpPortNum);
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
      error("ERROR connecting");
    set_nonblock(sockfd);
    printf("connected to server\n");
    return sockfd;
  }
  return -1;
}

int elCommRead(int fd)
{
  unsigned char c;
  unsigned int i;
  int rv;
  rv = read(fd, &c, 1); // read one byte
  i = c;          // convert byte to an int for return
  if (rv > 0)
    return i;     // return the character read
  if (rv < 0)
  {
    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
      perror("elCommRead() error:");
  }

  // return -1 or 0 either if we read nothing, or if read returned negative
  return rv;
}

int elCommWrite(int fd, uint8_t* data, int len)
{
  int rv;
  int length = len;
  int totalsent = 0;
  while (totalsent < length)
  {
    rv = write(fd, data + totalsent, length);
    if (rv < 0)
      perror("write(): error writing - trying again - ");
    else
      totalsent += rv;
  }
  return rv;
}

#endif
