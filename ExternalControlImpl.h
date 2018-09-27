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
	virtual bool OnGetUpdateArt(TObjectPoolIdx idx_local, cvTObjState* curState);
	virtual void OnPushUpdate(TObjectPoolIdx id_local, const cvTObjContInp* nextInput, const cvTObjState* nextState);
	virtual void OnPushUpdateArt(TObjectPoolIdx idx_local, const cvTObjState* curState);
	virtual void OnCreateADO(TObjectPoolIdx id_local
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l);
	virtual void OnDeleteADO(TObjectPoolIdx id_local);
	virtual bool Initialize(CHeaderDistriParseBlock& blk, CVED::ICvedDistri* pCvedDistri);
	virtual void UnInitialize();
private:
	virtual void CreateAdoStub(GlobalId id_global
							, const std::string& name
							, const cvTObjAttr& cAttr
							, const CPoint3D* cpInitPos
							, const CVector3D* cpInitTran
							, const CVector3D* cpInitLat);
	virtual void DeleteAdoStub(GlobalId id_global);
private:
	std::map<TObjectPoolIdx, GlobalId> m_mapLid2GidR;	//stores all the IDs of remote dynamic objects
	std::map<GlobalId, CDynObj*> m_mapGid2ObjR; 		//stores remote ADO stubs

	std::list<IP> m_multicastTo;
	IP m_selfIp;
	CVED::ICvedDistri* m_pCved;

	std::set<GlobalId> m_setAdosL;						//stores the IDs of local dynamic objects
	CDynObj* m_pedestrian;
};

#include "ExternalControlImpl.cpp"

#endif