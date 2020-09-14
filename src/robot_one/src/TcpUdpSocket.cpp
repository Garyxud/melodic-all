#include "TcpUdpSocket.h"

#ifdef WIN32
typedef int socklen_t;
#endif

TcpUdpSocket::TcpUdpSocket(int port, const char* address, bool udp, bool broadcast, bool reusesock, bool isServer, int timeout)
{
	connected = false;
	received[0] = '\0';
#ifdef WIN32
	retval = WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
	
	sockaddr_in addr;
	if(udp)
		sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	else
		sock = socket(AF_INET, SOCK_STREAM, 0);	

	#ifdef WIN32
	bool bOptVal = 1;
	int bOptLen = sizeof(bool);
	#else
	int OptVal = 1;
	#endif

	if (reusesock)
	#ifdef WIN32
		retval = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&bOptVal, bOptLen);
	#else
		retval = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &OptVal, sizeof(OptVal));
	#endif

	if(timeout > 0)
	{
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = timeout*1000;
		#ifdef WIN32
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv));
		#else
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		#endif
	}

	//set up bind address
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);

	if(isServer)
	{
		client = -1;
		retval = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
		retval = listen(sock, 1);
		connected = false;
	}
	else
	{
		client = -1;
		//set up address to use for sending
		memset(&outaddr, 0, sizeof(outaddr));
		outaddr.sin_family = AF_INET;
		outaddr.sin_addr.s_addr = inet_addr(address);
		outaddr.sin_port = htons(port);

		if (udp)
		{
			if (broadcast)
		#ifdef WIN32
				retval = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&bOptVal, bOptLen);
		#else
				retval = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &OptVal, sizeof(OptVal));
		#endif

			retval = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
		}
		else
		{
			retval = connect(sock, (struct sockaddr *)&outaddr, sizeof(outaddr));
		}
		if(udp)
			connected = retval != 0;
		else
			connected = retval == 0;
	}
}

TcpUdpSocket::~TcpUdpSocket()
{
#ifdef WIN32
	closesocket(sock);
	WSACleanup();
#else
	close(sock);
#endif
}

int TcpUdpSocket::getRetVal()
{
	return retval;
}

bool TcpUdpSocket::isConnected()
{
	return connected;
}

bool TcpUdpSocket::wait()
{
	if(client != -1)
		return true;
	int clientlen = sizeof(outaddr);
	client = accept(sock, (struct sockaddr *) &outaddr, (socklen_t*)&clientlen);
	return client >= 0;
}

void TcpUdpSocket::disconnect()
{
	if(client != -1)
	{
		#ifdef WIN32
			closesocket(client);
			WSACleanup();
		#else
			close(client);
		#endif
	}	
	client = -1;
}

int TcpUdpSocket::getAddress(const char * name, char * addr)
{
	struct hostent *hp;
	if ((hp = gethostbyname(name)) == NULL) return (0);
	strcpy(addr, inet_ntoa(*(struct in_addr*)(hp->h_addr)));
	return (1);
}

const char* TcpUdpSocket::getAddress(const char * name)
{
	if(client == -1)
	{
		struct hostent *hp;
		if ((hp = gethostbyname(name)) == NULL) return (0);
		strcpy(ip, inet_ntoa(*(struct in_addr*)(hp->h_addr)));
		return ip;
	}
	else
	{
		struct hostent *hostp = gethostbyaddr((const char *)&outaddr.sin_addr.s_addr, sizeof(outaddr.sin_addr.s_addr), AF_INET);
		if(hostp == NULL)
			return 0;
		strcpy(ip, inet_ntoa(outaddr.sin_addr));
		strcpy(received, ip);
		return ip;
	}
}

long TcpUdpSocket::receive(char* msg, int msgsize)
{
	if(client == -1)
	{
		struct sockaddr_in sender;
		socklen_t sendersize = sizeof(sender);
		int retval = recvfrom(sock, msg, msgsize, 0, (struct sockaddr *)&sender, &sendersize);
		strcpy(received, inet_ntoa(sender.sin_addr));
		return retval;
	}
	else
	{
		#ifdef WIN32
			int retval = recv(client, msg, msgsize, 0);
		#else
			int retval = read(client, msg, msgsize);
		#endif
		return retval;
	}
}

char* TcpUdpSocket::received_from()
{
	return received;
}

long TcpUdpSocket::sendTo(const char* msg, int msgsize)
{
	if(client == -1)
		return sendto(sock, msg, msgsize, 0, (struct sockaddr *)&outaddr, sizeof(outaddr));
	else
		#ifdef WIN32
			return send(client, msg, msgsize, 0);
		#else
			return write(client, msg, msgsize);
		#endif
}

long TcpUdpSocket::sendTo(const char* msg, int msgsize, const char* addr)
{
	outaddr.sin_addr.s_addr = inet_addr(addr);
	return sendto(sock, msg, msgsize, 0, (struct sockaddr *)&outaddr, sizeof(outaddr));
}