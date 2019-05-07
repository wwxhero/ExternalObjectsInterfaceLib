#ifndef __INETWORKDYNAMIC
#define __INETWORKDYNAMIC
#pragma once
#include <set>
#include <list>
#include "cveddecl.h"
#include "objlayout.h"
typedef unsigned long IP;
struct cvTObjStateBuf;
//the implementation of this interface should maintain a queue datastructure for each IP contains received data
//once receive is called, then it pops the head element if there is,
//(assumption: the earlier received data with the smallest frame number, this assumption is true for most of the network transportation cases, with few exceptions)
typedef struct {
	IP owner;
	TObjectPoolIdx objId;
} GlobalId;

inline bool operator < (GlobalId id1, GlobalId id2)
{
	return id1.owner < id2.owner
		|| (id1.owner == id2.owner && id1.objId < id2.objId);
}

class INetworkDynamic {
public:
	virtual void NetworkInitialize(const std::list<IP>& senders, const std::list<IP>& receivers, int port, IP self) = 0;
	virtual void NetworkUninitialize() = 0;
	virtual void PreDynaCalc() = 0;
	virtual void Send(IP ip, GlobalId id_global, const cvTObjState* s) = 0;
	virtual void SendArt(IP ip, GlobalId id_global, const cvTObjState* s) = 0;
	virtual void SendArt(IP ip, GlobalId id_global, GlobalId id_parent, const cvTObjState* s) = 0;
	virtual bool Receive(GlobalId id_global, cvTObjState* s) = 0;
	virtual bool ReceiveArt(GlobalId id_global, cvTObjState* s) = 0;
	virtual bool ReceiveArt(GlobalId id_global, GlobalId id_parent, cvTObjState* s) = 0;
	virtual void Notify_OnNewAdo(GlobalId id
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l) = 0;
	virtual void Notify_OnDelAdo(GlobalId id) = 0;
	virtual void PostDynaCalc() = 0;
	virtual void GetLocalhostIps(std::set<IP>& setIps) = 0;
};

#endif