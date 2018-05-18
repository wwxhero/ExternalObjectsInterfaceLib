#pragma once
#ifndef __EXTERNAL_CONTROL_IMPL_H
#define __EXTERNAL_CONTROL_IMPL_H
#include "ExternalControlInterface.h"
#include "NetworkDynamicWin.h"


template<class TNetworkImpl>
class CExternalObjectControlImpl : public CVED::IExternalObjectControl
								 , public CNetworkDynamicWin<TNetworkImpl>
{

public:
	CExternalObjectControlImpl();
	virtual ~CExternalObjectControlImpl();
	virtual void PreUpdateDynamicModels();
	virtual void PostUpdateDynamicModels();
	virtual bool OnGetUpdate(TObjectPoolIdx id_local, cvTObjContInp* curInput, cvTObjState* curState);
	virtual void OnPushUpdate(TObjectPoolIdx id_local, const cvTObjContInp* nextInput, const cvTObjState* nextState);
	virtual bool Initialize(CHeaderDistriParseBlock& blk, CVED::CCved* pCved);
	virtual void UnInitialize(CVED::CCved* pCved);
private:
	typedef struct _SEG
	{
	    IP ip;
	    IP mask;
	    IP Group()
	    {
			IP g = ip & mask;
			g = g | (~mask);
			return g;
	    }
	} SEG;
	void InitIpclusters(const std::list<SEG>& ips, std::list<IP>& clusters);
	void BroadCastObj(TObjectPoolIdx id_local, const cvTObjStateBuf* sb);
	static CVED::CDynObj* CreatePeerDriver(CHeaderDistriParseBlock& blk, CVED::CCved* cved, cvEObjType runAs);
private:
	std::map<TObjectPoolIdx, GlobalId> m_mapLid2Gid;

	std::list<IP> m_ipClusters;
	std::list<CVED::CDynObj*> m_lstPeers;
	IP m_selfIp;
};

#include "ExternalControlImpl.cpp"

#endif