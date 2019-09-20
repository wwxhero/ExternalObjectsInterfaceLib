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
	virtual void OnTelePDO(TObjContInpPoolIdx id_local, const CPoint3D& pos, const CVector3D& tan, const CVector3D& lat);
	virtual bool Initialize(CHeaderDistriParseBlock& blk, CVED::ICvedDistri* pCvedDistri);
	virtual void UnInitialize();
private:
	virtual void OnNotify_OnNewAdo(GlobalId id_global
							, const std::string& name
							, const cvTObjAttr& cAttr
							, const CPoint3D* cpInitPos
							, const CVector3D* cpInitTran
							, const CVector3D* cpInitLat);
	virtual void OnNotify_OnDelAdo(GlobalId id_global);
	virtual void OnNotify_OnTelePdo(GlobalId id_global, CPoint3D* p, CVector3D* t, CVector3D* l);
private:
	std::map<TObjectPoolIdx, GlobalId> m_mapLid2GidR;	//stores all the IDs of remote dynamic objects
	std::map<GlobalId, CVED::CDynObj*> m_mapGid2ObjR; 		//stores remote dynamic objects

	std::list<IP> m_multicastTo;
	IP m_selfIp;
	CVED::ICvedDistri* m_pCved;

	std::set<GlobalId> m_setAdosL;						//stores the IDs of local dynamic objects
	CVED::CExternalAvatarObj* m_pedestrian;
};

#include "ExternalControlImpl.cpp"

#endif