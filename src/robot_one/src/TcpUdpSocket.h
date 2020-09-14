#ifndef TCPUDPSOCKET_H_INCLUDED
#define TCPUDPSOCKET_H_INCLUDED

#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#define TRUE 1
#define FALSE 0
#endif


class TcpUdpSocket
{
private:
#ifdef WIN32
	WSAData wsaData;
	SOCKET sock;
	SOCKET client;
#else
	int sock;
	int client;
#endif
	bool connected;
	long retval;
	sockaddr_in outaddr;
	char ip[30];
	char received[30];


public:
	TcpUdpSocket(int port, const char* address, bool udp, bool Datagram = true, bool reusesock = true, bool isServer = false, int timeout = 0);
	~TcpUdpSocket();

	int getRetVal();
	bool isConnected();

	bool wait();
	void disconnect();

	long receive(char* msg, int msgsize);
	char* received_from();
	long sendTo(const char* msg, int msgsize);
	long sendTo(const char* msg, int msgsize, const char* name);
	int getAddress(const char * name, char * addr);
	const char* getAddress(const char * name);

};



#endif // DATAGRAMSOCKET_H_INCLUDED