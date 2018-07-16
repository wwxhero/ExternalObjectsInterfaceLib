#include "cvedstrc.h"
#include "dynobj.h"
#include "headerparseblock.h"
#include "roadpos.h"
#include "utility.h"
#include "cveddistri.h"

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
	GlobalId id_global = (*it).second;
	const cvTObjStateBuf* sbAlt = NULL;
	bool recieved = Receive(id_global, sbAlt);
	if (recieved)
	{
		cvTObjState::ExternalDriverState* s_np = (cvTObjState::ExternalDriverState*)(&sbAlt->state.externalDriverState);
		cvTObjState::ExternalDriverState* s_n  = (cvTObjState::ExternalDriverState*)(&curState->externalDriverState);
		for (int i = 0; i < 2; i ++)
		{
			s_np->boundBox[i].x = s_n->boundBox[i].x - s_n->position.x + s_np->position.x;
			s_np->boundBox[i].y = s_n->boundBox[i].y - s_n->position.y + s_np->position.y;
			s_np->boundBox[i].z = s_n->boundBox[i].z - s_n->position.z + s_np->position.z;
		}
		memcpy(curInput, &sbAlt->contInp, sizeof(cvTObjContInp));
		memcpy(curState, &sbAlt->state, sizeof(cvTObjState));
	}
	const TCHAR* recFlag[] = {
		TEXT("NReceived")
		, TEXT("Received")
	};
	const unsigned char* seg = (const unsigned char*)&id_global.owner;
	int idx = recieved? 1: 0;
	TRACE(TEXT("OnGetUpdate %s id:%d from ip:[%d.%d.%d.%d]")
								TEXT("\n\t position: [%E,%E,%E]")
								TEXT("\n\t tangent: [%E,%E,%E]")
								TEXT("\n\t lateral: [%E,%E,%E]\n")
									, recFlag[idx], id_local, seg[0], seg[1], seg[2], seg[3]
									, curState->anyState.position.x, curState->anyState.position.y, curState->anyState.position.z
									, curState->anyState.tangent.i, curState->anyState.tangent.j, curState->anyState.tangent.k
									, curState->anyState.lateral.i, curState->anyState.lateral.j, curState->anyState.lateral.k );
	return recieved;
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::OnPushUpdate(TObjectPoolIdx id_local, const cvTObjContInp* nextInput, const cvTObjState* nextState)
{
	cvTObjStateBuf sb;
	sb.contInp = *nextInput;
	sb.state = *nextState;
	BroadCastObj(id_local, &sb);
	const struct cvTObjState::ExternalDriverState& s = nextState->externalDriverState;
	TRACE(TEXT("OnPushUpdate id:%d, \n\t position: [%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]")
							TEXT(", \n\t bbox: [%E,%E,%E], [%E,%E,%E]")
							TEXT(", \n\t vel: [%E]")
							TEXT(", \n\t visualState: [%E] audioState: [%E]")
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
bool CExternalObjectControlImpl<TNetworkImpl>::Initialize(CHeaderDistriParseBlock& hBlk, CVED::ICvedDistri* pCvedDistri)
{
	std::set<IP> localhostIps;
	GetLocalhostIps(localhostIps);

	std::list<SEG> neighborsTo;
	std::list<IP> neighborsFrom;
	int id_local = 0; //cved object id
	int numSelf = 0;
	//edo_controller, ado_controller
	cvTObjState state0 = {0};
	cvTObjContInp inp0 = {0};

	do
	{
		std::string ipStr = hBlk.GetIPV4();
		std::string ipMask = hBlk.GetIPMask();
		IP simIP = inet_addr(ipStr.c_str());
		IP simMask = inet_addr(ipMask.c_str());
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
			SEG seg = {simIP, simMask};
			neighborsTo.push_back(seg);
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
			CVED::CDynObj* peerObj = pCvedDistri->LocalCreateExtObj(hBlk);
			id_local = peerObj->GetId();
			GlobalId id_global = {simIP, 0};
			m_mapGid2ObjR[id_global] = peerObj;
			m_mapLid2GidR.insert(std::pair<TObjectPoolIdx, GlobalId>(id_local, id_global));
		}
		else if (ownEdoBlk)
		{
			CVED::CDynObj* psudoOwn = pCvedDistri->LocalCreateExtObj(hBlk);
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
			//fixme: pedestrain object is considered as a vehicle
			m_pedestrian = pCvedDistri->LocalCreatePedObj(hBlk);
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
		else if (peerPedBlk) //runs for nads simulator
		{
			//fixme: a terminal computer can only have 1 avatar
			CVED::CDynObj* avatar = pCvedDistri->LocalCreatePedObj(hBlk);
			GlobalId id_global = {simIP, 0};
			m_mapGid2ObjR[id_global] = avatar;
			int id_local = avatar->GetId();
			m_mapLid2GidR.insert(std::pair<TObjectPoolIdx, GlobalId>(id_local, id_global));
		}

	} while (hBlk.NextExternalBlk());

	bool ok = (numSelf > 0); //scene includes this computer as a terminal
	if (ok)
	{
		InitIpclusters(neighborsTo, m_ipClusters);
		NetworkInitialize(m_ipClusters, neighborsFrom, hBlk.GetPort(), m_selfIp);
		m_pCved = pCvedDistri;

		if (edo_controller == c_type
			|| ped_controller == c_type)
			OnPushUpdate(0, &inp0, &state0);
	}
	return ok;
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::InitIpclusters(const std::list<SEG>& ips, std::list<IP>& clusters)
{
	typedef struct _Cluster {
		IP ip;
		int mag;
	} Cluster;
	std::map<IP, Cluster> group2cluster;
	for (std::list<SEG>::const_iterator it = ips.begin(); it != ips.end(); it ++)
	{
		SEG seg = *it;
		IP gip = seg.Group();
		Cluster c;
		std::map<IP, Cluster>::iterator itCluster;
		if ((itCluster = group2cluster.find(gip)) == group2cluster.end())
		{
			c.ip = seg.ip;
			c.mag = 1;
		}
		else
		{
			c.mag = (*itCluster).second.mag + 1;
		}
		group2cluster[gip] = c;
	}
	for (std::map<IP, Cluster>::iterator it = group2cluster.begin()
		; it != group2cluster.end()
		; it ++)
	{
		std::pair<IP, Cluster> p = *it;
		IP ip;
		if (1) //p.second.mag > 1) vrlink support for unicast is buggy, thus we just use broad cast
			ip = p.first;
		else
			ip = p.second.ip;
		clusters.push_back(ip);
	}
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
	m_ipClusters.clear();
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
void CExternalObjectControlImpl<TNetworkImpl>::BroadCastObj(TObjectPoolIdx id_local, const cvTObjStateBuf* sb)
{
	GlobalId id_global = {m_selfIp, id_local};
	for (std::list<IP>::iterator it = m_ipClusters.begin(); it != m_ipClusters.end(); it ++)
	{
		IP ipCluster = *it;
		unsigned char* ipv4 = (unsigned char*)&ipCluster;
		TRACE(TEXT("Send to(IPV4):%d.%d.%d.%d\n")
										, ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
		Send(ipCluster, id_global, *sb);
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
	CDynObj* obj = m_pCved->LocalCreateDynObj(name, eCV_VEHICLE, cAttr, cpInitPos, cpInitTran, cpInitLat);
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
