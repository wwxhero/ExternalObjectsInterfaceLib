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
	virtual void OnCreateADO(TObjectPoolIdx id_local
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l);
	virtual void OnDeleteADO(TObjectPoolIdx id_local);
	virtual bool Initialize(CHeaderDistriParseBlock& blk, CVED::CCvedDistri* pCved);
	virtual void UnInitialize();
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

	virtual void CreateAdoStub(GlobalId id_global
							, const std::string& name
							, const cvTObjAttr& cAttr
							, const CPoint3D* cpInitPos
							, const CVector3D* cpInitTran
							, const CVector3D* cpInitLat);
	virtual void DeleteAdoStub(GlobalId id_global);
private:
	std::map<TObjectPoolIdx, GlobalId> m_mapLid2Gid;
	std::map<GlobalId, CDynObj*> m_mapGid2Ado;

	std::list<IP> m_ipClusters;
	std::list<CVED::CDynObj*> m_lstPeers;
	IP m_selfIp;
	CVED::CCvedDistri* m_pCved;

	std::set<GlobalId> m_setAdos;
};

#include "ExternalControlImpl.cpp"

#endif