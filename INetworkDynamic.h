#ifndef __INETWORKDYNAMIC
#define __INETWORKDYNAMIC
#pragma once
#include <set>
#include <list>
typedef unsigned long IP;
struct cvTObjStateBuf;
//the implementation of this interface should maintain a queue datastructure for each IP contains received data
//once receive is called, then it pops the head element if there is,
//(assumption: the earlier received data with the smallest frame number, this assumption is true for most of the network transportation cases, with few exceptions)
class INetworkDynamic {
public:
	virtual void NetworkInitialize(const std::list<IP>& senders, const std::list<IP>& receivers, int port, IP self) = 0;
	virtual void NetworkUninitialize() = 0;
	virtual void PreDynaCalc() = 0;
	virtual void Send(IP ip, const cvTObjStateBuf& sb) = 0;
	virtual bool Receive(IP ip, const cvTObjStateBuf*& sb) = 0;
	virtual void PostDynaCalc() = 0;
	virtual void GetLocalhostIps(std::set<IP>& setIps) = 0;
};

#endif