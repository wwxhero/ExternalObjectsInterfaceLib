#include "cvedstrc.h"
#include "dynobj.h"
#include "headerparseblock.h"
#include "roadpos.h"
#include "utility.h"
#include "cveddistri.h"
#include "vrlinkMath.h"

template<class TNetworkImpl>
CExternalObjectControlImpl<TNetworkImpl>::CExternalObjectControlImpl()
	: m_pCved(NULL)
	, m_selfIp(0)
	, m_pedestrian(NULL)
{
}

template<class TNetworkImpl>
CExternalObjectControlImpl<TNetworkImpl>::~CExternalObjectControlImpl()
{
}

template<class TNetworkImpl>
bool CExternalObjectControlImpl<TNetworkImpl>::OnGetUpdate(TObjectPoolIdx id_local, cvTObjContInp* curInput, cvTObjState* curState)
{
	std::map<TObjectPoolIdx, GlobalId>::iterator it = m_mapLid2GidR.find(id_local);
	if (it == m_mapLid2GidR.end())
	{
		//assert(0);
		return false;
	}
	cvTObjState::AnyObjState* s = &(curState->anyState);
	const TVector3D boxOffset[2] = {
			{
				s->boundBox[0].x - s->position.x
				, s->boundBox[0].y - s->position.y
				, s->boundBox[0].z - s->position.z
			}
			,{
				s->boundBox[1].x - s->position.x
				, s->boundBox[1].y - s->position.y
				, s->boundBox[1].z - s->position.z
			}
	};
	GlobalId id_global = (*it).second;
	bool recieved = Receive(id_global, curState);
	if (recieved)
	{
		for (int i = 0; i < 2; i ++)
		{
			s->boundBox[i].x = boxOffset[i].i + s->position.x;
			s->boundBox[i].y = boxOffset[i].j + s->position.y;
			s->boundBox[i].z = boxOffset[i].k + s->position.z;
		}
	}
#ifdef _DEBUG
	const TCHAR* recFlag[] = {
		TEXT("NReceived")
		, TEXT("Received")
	};
	const unsigned char* seg = (const unsigned char*)&id_global.owner;
	int idx = recieved? 1: 0;
	//curState->externalDriverState.visualState = 2;
	const struct cvTObjState::ExternalDriverState& es = curState->externalDriverState;
	TRACE(TEXT("OnGetUpdate %s id:%d from ip:[%d.%d.%d.%d]")
							TEXT(", \n\t position: [%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]")
							TEXT(", \n\t bbox: [%E,%E,%E], [%E,%E,%E]")
							TEXT(", \n\t vel: [%E]")
							TEXT(", \n\t visualState: [%d] audioState: [%d]")
							TEXT(", \n\t acc: [%E]")
							TEXT(", \n\t sus4: [%E, %E, %E, %E]")
							TEXT(", \n\t velBrake: [%E]")
							TEXT(", \n\t latAccel: [%E]")
							TEXT(", \n\t Fidelity: [%d]")
							TEXT(", \n\t angularVel: [%E, %E, %E]\n")
										, recFlag[idx], id_local, seg[0], seg[1], seg[2], seg[3]
										, es.position.x, es.position.y, es.position.z
										, es.tangent.i, es.tangent.j, es.tangent.k
										, es.lateral.i, es.lateral.j, es.lateral.k
										, es.boundBox[0].x, es.boundBox[0].y, es.boundBox[0].z
										, es.boundBox[1].x, es.boundBox[1].y, es.boundBox[1].z
										, es.vel
										, es.visualState, es.audioState
										, es.acc
										, es.suspStif, es.suspDamp, es.tireStif, es.tireDamp
										, es.velBrake
										, es.latAccel
										, es.dynaFidelity
										, es.angularVel.i, es.angularVel.j, es.angularVel.k);
#endif
	return recieved;
}

template<class TNetworkImpl>
bool CExternalObjectControlImpl<TNetworkImpl>::OnGetUpdateArt(TObjectPoolIdx id_local, cvTObjState* curState)
{
	std::map<TObjectPoolIdx, GlobalId>::iterator it = m_mapLid2GidR.find(id_local);
	if (it == m_mapLid2GidR.end())
	{
		//assert(0);
		return false;
	}
	cvTObjState::AnyObjState* s = &(curState->anyState);
	const TVector3D boxOffset[2] = {
			{
				s->boundBox[0].x - s->position.x
				, s->boundBox[0].y - s->position.y
				, s->boundBox[0].z - s->position.z
			}
			,{
				s->boundBox[1].x - s->position.x
				, s->boundBox[1].y - s->position.y
				, s->boundBox[1].z - s->position.z
			}
	};
	GlobalId id_global = (*it).second;
	bool recieved = ReceiveArt(id_global, curState);
	if (recieved)
	{
		for (int i = 0; i < 2; i ++)
		{
			s->boundBox[i].x = boxOffset[i].i + s->position.x;
			s->boundBox[i].y = boxOffset[i].j + s->position.y;
			s->boundBox[i].z = boxOffset[i].k + s->position.z;
		}
	}

#ifdef _DEBUG
	const TCHAR* recFlag[] = {
		TEXT("NReceived")
		, TEXT("Received")
	};
	const unsigned char* seg = (const unsigned char*)&id_global.owner;
	int idx = recieved? 1: 0;
	//curState->externalDriverState.visualState = 2;
	const struct cvTObjState::AvatarState& s_a = curState->avatarState;
	TRACE(TEXT("OnGetUpdate %s id:%d from ip:[%d.%d.%d.%d]")
							TEXT(", \n\t position: [%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]")
							TEXT(", \n\t bbox: [%E,%E,%E], [%E,%E,%E]")
							TEXT(", \n\t vel: [%E]")
							TEXT(", \n\t acc: [%E]")
							TEXT(", \n\t latAccel: [%E]")
							TEXT(", \n\t angularVel: [%E, %E, %E]\n")
										, recFlag[idx], id_local, seg[0], seg[1], seg[2], seg[3]
										, s_a.position.x, s_a.position.y, s_a.position.z
										, s_a.tangent.i, s_a.tangent.j, s_a.tangent.k
										, s_a.lateral.i, s_a.lateral.j, s_a.lateral.k
										, s_a.boundBox[0].x, s_a.boundBox[0].y, s_a.boundBox[0].z
										, s_a.boundBox[1].x, s_a.boundBox[1].y, s_a.boundBox[1].z
										, s_a.vel
										, s_a.acc
										, s_a.latAccel
										, s_a.angularVel.i, s_a.angularVel.j, s_a.angularVel.k);
	const char** szNames = NULL;
	unsigned int numNames = 0;
	CDynObj* pDynObj = m_mapGid2ObjR[id_global];
	CArtiJoints::BFTAlloc(pDynObj->GetName(), &szNames, &numNames);
	TVector3D* angles = new TVector3D[numNames];
	CArtiJoints::BFTGetJoints(curState, angles, numNames);
	TRACE(TEXT(", \n\t joints:"));
	for (int i_n = 0; i_n < numNames; i_n ++)
	{
		TRACE(TEXT(", \n\t\t%d:[%s]=<%d, %d, %d>"), i_n, szNames[i_n], (int)rad2deg(angles[i_n].i), (int)rad2deg(angles[i_n].j), (int)rad2deg(angles[i_n].k));
	}
	delete [] angles;
	CArtiJoints::BFTFree(szNames, numNames);
#endif
	return recieved;
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::OnPushUpdate(TObjectPoolIdx id_local, const cvTObjContInp* nextInput, const cvTObjState* nextState)
{

	GlobalId id_global = {m_selfIp, id_local};
	for (std::list<IP>::iterator it = m_multicastTo.begin(); it != m_multicastTo.end(); it ++)
	{
		IP ipCluster = *it;
		unsigned char* ipv4 = (unsigned char*)&ipCluster;
		TRACE(TEXT("Send to(IPV4):%d.%d.%d.%d\n")
										, ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
		Send(ipCluster, id_global, nextState);
	}

	const struct cvTObjState::ExternalDriverState& s = nextState->externalDriverState;
	TRACE(TEXT("OnPushUpdate id:%d, \n\t position: [%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]")
							TEXT(", \n\t bbox: [%E,%E,%E], [%E,%E,%E]")
							TEXT(", \n\t vel: [%E]")
							TEXT(", \n\t visualState: [%d] audioState: [%d]")
							TEXT(", \n\t acc: [%E]")
							TEXT(", \n\t sus4: [%E, %E, %E, %E]")
							TEXT(", \n\t velBrake: [%E]")
							TEXT(", \n\t latAccel: [%E]")
							TEXT(", \n\t Fidelity: [%d]")
							TEXT(", \n\t angularVel: [%E, %E, %E]\n")
										, id_local
										, s.position.x, s.position.y, s.position.z
										, s.tangent.i, s.tangent.j, s.tangent.k
										, s.lateral.i, s.lateral.j, s.lateral.k
										, s.boundBox[0].x, s.boundBox[0].y, s.boundBox[0].z
										, s.boundBox[1].x, s.boundBox[1].y, s.boundBox[1].z
										, s.vel
										, s.visualState, s.audioState
										, s.acc
										, s.suspStif, s.suspDamp, s.tireStif, s.tireDamp
										, s.velBrake
										, s.latAccel
										, s.dynaFidelity
										, s.angularVel.i, s.angularVel.j, s.angularVel.k);
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::OnPushUpdateArt(TObjectPoolIdx id_local, const cvTObjState* nextState)
{
	//todo: push articulated structure data stored from nextState
	GlobalId id_global = {m_selfIp, id_local};
	for (std::list<IP>::iterator it = m_multicastTo.begin(); it != m_multicastTo.end(); it ++)
	{
		IP ipCluster = *it;
		unsigned char* ipv4 = (unsigned char*)&ipCluster;
		TRACE(TEXT("Send articulated to(IPV4):%d.%d.%d.%d\n")
										, ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
		SendArt(ipCluster, id_global, nextState);
	}
#ifdef _DEBUG
	const struct cvTObjState::AvatarState& s = nextState->avatarState;
	TRACE(TEXT("OnPushUpdateArt id:%d, \n\t position: [%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]")
							TEXT(", \n\t bbox: [%E,%E,%E], [%E,%E,%E]")
							TEXT(", \n\t vel: [%E]")
							TEXT(", \n\t visualState: [%d] audioState: [%d]")
							TEXT(", \n\t acc: [%E]")
							TEXT(", \n\t latAccel: [%E]")
							TEXT(", \n\t angularVel: [%E, %E, %E]")
										, id_local
										, s.position.x, s.position.y, s.position.z
										, s.tangent.i, s.tangent.j, s.tangent.k
										, s.lateral.i, s.lateral.j, s.lateral.k
										, s.boundBox[0].x, s.boundBox[0].y, s.boundBox[0].z
										, s.boundBox[1].x, s.boundBox[1].y, s.boundBox[1].z
										, s.vel
										, s.visualState, s.audioState
										, s.acc
										, s.latAccel
										, s.angularVel.i, s.angularVel.j, s.angularVel.k);
	CExternalAvatarObj* pAvatar = m_pedestrian;

	const char** szNames = NULL;
	unsigned int numNames = 0;
	CArtiJoints::BFTAlloc(pAvatar->GetName(), &szNames, &numNames);
	TVector3D* angles = new TVector3D[numNames];
	CArtiJoints::BFTGetJoints(nextState, angles, numNames);
	TRACE(TEXT(", \n\t joints:"));
	for (int i_n = 0; i_n < numNames; i_n ++)
	{
		TRACE(TEXT(", \n\t\t%d:[%s]=<%d, %d, %d>"), i_n, szNames[i_n], (int)rad2deg(angles[i_n].i), (int)rad2deg(angles[i_n].j), (int)rad2deg(angles[i_n].k));
	}
	TRACE(TEXT("\n"));
	delete [] angles;
	CArtiJoints::BFTFree(szNames, numNames);
#endif
}


template<class TNetworkImpl>
bool CExternalObjectControlImpl<TNetworkImpl>::Initialize(CHeaderDistriParseBlock& hBlk, CVED::ICvedDistri* pCvedDistri)
{
	std::set<IP> localhostIps;
	GetLocalhostIps(localhostIps);

	std::list<IP> neighborsFrom;
	int id_local = 0; //cved object id
	int numSelf = 0;
	//edo_controller, ado_controller
	cvTObjState state0 = {0};
	cvTObjContInp inp0 = {0};

	do
	{
		std::string ipStr = hBlk.GetIPV4();

		IP simIP = inet_addr(ipStr.c_str());
		bool selfBlk = (localhostIps.end() != localhostIps.find(simIP))
						&& c_type == hBlk.GetType();
		bool neighborBlk = !selfBlk;
		bool peerEdoBlk = (edo_controller == hBlk.GetType()
						&& (edo_controller != c_type || !selfBlk));
		bool ownEdoBlk = (edo_controller == hBlk.GetType() && selfBlk);
		bool peerAdoBlk = false; //due to current design, peer ado will constantly be false
		bool ownAdoBlk = (ado_controller == hBlk.GetType() && selfBlk);
		bool peerPedBlk = (ped_controller == hBlk.GetType()
						&& (ped_controller != c_type || !selfBlk));
		bool ownPedBlk = (ped_controller == hBlk.GetType() && selfBlk);

		if (neighborBlk)
		{
			neighborsFrom.push_back(simIP);
		}
		else //selfBlk
		{
			m_selfIp = simIP;
			hBlk.TagLocalhost();
			numSelf ++;
		}

		if (peerEdoBlk)
		{
			CVED::CDynObj* peerObj = pCvedDistri->LocalCreateEDO(hBlk);
			id_local = peerObj->GetId();
			GlobalId id_global = {simIP, 0};
			m_mapGid2ObjR[id_global] = peerObj;
			m_mapLid2GidR.insert(std::pair<TObjectPoolIdx, GlobalId>(id_local, id_global));
		}
		else if (ownEdoBlk)
		{
			CVED::CDynObj* psudoOwn = pCvedDistri->LocalCreateEDO(hBlk);
			CPoint3D pos = psudoOwn->GetPos();
			CVector3D tan = psudoOwn->GetTan();
			CVector3D lat = psudoOwn->GetLat();
			TPoint3D p = {pos.m_x, pos.m_y, pos.m_z};
			TVector3D t = {tan.m_i, tan.m_j, tan.m_k};
			TVector3D l = {lat.m_i, lat.m_j, lat.m_k};
			state0.externalDriverState.position = p;
			state0.externalDriverState.tangent = t;
			state0.externalDriverState.lateral = l;
			pCvedDistri->LocalDeleteDynObj(psudoOwn);
		}
		else if (ownPedBlk) //runs for hank simulator
		{
			m_pedestrian = static_cast<CExternalAvatarObj*>(pCvedDistri->LocalCreatePDO(hBlk, true));
			CPoint3D pos = m_pedestrian->GetPos();
			CVector3D tan = m_pedestrian->GetTan();
			CVector3D lat = m_pedestrian->GetLat();
			TPoint3D p = {pos.m_x, pos.m_y, pos.m_z};
			TVector3D t = {tan.m_i, tan.m_j, tan.m_k};
			TVector3D l = {lat.m_i, lat.m_j, lat.m_k};
			state0.externalDriverState.position = p;
			state0.externalDriverState.tangent = t;
			state0.externalDriverState.lateral = l;
		}
		else if (peerPedBlk)
		{
			//fixme: a terminal computer can only have 1 avatar
			CVED::CDynObj* avatar = pCvedDistri->LocalCreatePDO(hBlk, false);
			GlobalId id_global = {simIP, 0};
			m_mapGid2ObjR[id_global] = avatar;
			int id_local = avatar->GetId();
			m_mapLid2GidR.insert(std::pair<TObjectPoolIdx, GlobalId>(id_local, id_global));
		}

	} while (hBlk.NextExternalBlk());

	bool ok = (numSelf > 0); //scene includes this computer as a terminal
	if (ok)
	{
		IP mcIP = inet_addr(hBlk.GetMCIP().c_str());
		m_multicastTo.push_back(mcIP);
		NetworkInitialize(m_multicastTo, neighborsFrom, hBlk.GetPort(), m_selfIp);
		m_pCved = pCvedDistri;

		if (edo_controller == c_type
			|| ped_controller == c_type)
			OnPushUpdate(0, &inp0, &state0);
	}
	return ok;
}



template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::UnInitialize()
{
	for (std::set<GlobalId>::iterator it = m_setAdosL.begin()
		; it != m_setAdosL.end()
		; ++ it)
	{
		GlobalId id = *it;
		TNetworkImpl::Notify_OnDelAdo(id);
	}
	m_setAdosL.clear();
	NetworkUninitialize();
	m_multicastTo.clear();
	m_mapLid2GidR.clear();

	for (std::map<GlobalId, CDynObj*>::iterator it = m_mapGid2ObjR.begin()
		; it != m_mapGid2ObjR.end()
		; it ++)
	{
		std::pair<GlobalId, CDynObj*> p = *it;
		m_pCved->LocalDeleteDynObj(p.second);
	}
	m_mapGid2ObjR.clear();

	if (NULL != m_pedestrian)
	{
		m_pCved->LocalDeleteDynObj(m_pedestrian);
		m_pedestrian = NULL;
	}
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::PreUpdateDynamicModels()
{
	PreDynaCalc();
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::PostUpdateDynamicModels()
{
	PostDynaCalc();
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::CreateAdoStub(GlobalId id_global
													, const std::string& name
													, const cvTObjAttr& cAttr
													, const CPoint3D* cpInitPos
													, const CVector3D* cpInitTran
													, const CVector3D* cpInitLat)
{
	CDynObj* obj = m_pCved->LocalCreateADO(name, cAttr, cpInitPos, cpInitTran, cpInitLat);
	TObjectPoolIdx id_local = obj->GetId();
	m_mapLid2GidR[id_local] = id_global;
	m_mapGid2ObjR[id_global] = obj;
	TRACE(TEXT("EDO Ctrl: Create ADO %d\n"), id_local);
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::DeleteAdoStub(GlobalId id_global)
{
	std::map<GlobalId, CDynObj*>::iterator it = m_mapGid2ObjR.find(id_global);
	if (it != m_mapGid2ObjR.end())
	{
		CDynObj* obj = (*it).second;
		TObjectPoolIdx id_local = obj->GetId();
		m_pCved->LocalDeleteDynObj(obj);
		m_mapGid2ObjR.erase(it);
		m_mapLid2GidR.erase(id_local);
		TRACE(TEXT("EDO Ctrl: Delete ADO %d\n"), id_local);
	}
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::OnCreateADO(TObjectPoolIdx id_local
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l)
{
	GlobalId id = {m_selfIp, id_local};
	TNetworkImpl::Notify_OnNewAdo(id, szName, cAttr, pos, t, l);
	TRACE(TEXT("ADO Ctrl: Create ADO %d\n"), id_local);
	m_setAdosL.insert(id);
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::OnDeleteADO(TObjectPoolIdx id_local)
{
	GlobalId id = {m_selfIp, id_local};
	TNetworkImpl::Notify_OnDelAdo(id);
	TRACE(TEXT("ADO Ctrl: Delete ADO %d\n"), id_local);
	m_setAdosL.erase(id);
}
