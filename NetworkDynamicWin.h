#pragma once
#include "inetworkdynamic.h"
#include <ws2tcpip.h>
#define MAX_HOSTNAMELEN 256
template<typename INetworkDynamicImpl>
class CNetworkDynamicWin : public INetworkDynamicImpl
{
public:
	CNetworkDynamicWin(void)
	{
		WSADATA wsaData;
		int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		assert(0 == iResult);
	}
	virtual ~CNetworkDynamicWin(void)
	{
		WSACleanup();
	}
	virtual void GetLocalhostIps(std::set<IP>& setIps)
	{
		char hostName[MAX_HOSTNAMELEN] = {0};
		struct addrinfo hints;
		struct addrinfo *result = NULL;
		// Setup the hints address info structure
		// which is passed to the getaddrinfo() function
		ZeroMemory( &hints, sizeof(hints) );
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		if ( 0 == gethostname(hostName, MAX_HOSTNAMELEN)
			&& 0 == getaddrinfo(hostName, NULL, &hints, &result) ) {
			for(struct addrinfo *ptr=result; ptr != NULL ;ptr=ptr->ai_next) {
				if (AF_INET == ptr->ai_family) {
					struct sockaddr_in  *sockaddr_ipv4 = (struct sockaddr_in *) ptr->ai_addr;
					setIps.insert(sockaddr_ipv4->sin_addr.S_un.S_addr);
				}
			}
		}

		freeaddrinfo(result);
	}
};

