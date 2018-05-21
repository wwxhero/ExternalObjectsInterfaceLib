#include "cvedstrc.h"
#include "dynobj.h"
#include "headerparseblock.h"
#include "roadpos.h"
#include "utility.h"

template<class TNetworkImpl>
CExternalObjectControlImpl<TNetworkImpl>::CExternalObjectControlImpl()
	: m_selfIp(0)
	, m_pCved(NULL)
{
}

template<class TNetworkImpl>
CExternalObjectControlImpl<TNetworkImpl>::~CExternalObjectControlImpl()
{
}

template<class TNetworkImpl>
bool CExternalObjectControlImpl<TNetworkImpl>::OnGetUpdate(TObjectPoolIdx id_local, cvTObjContInp* curInput, cvTObjState* curState)
{
	std::map<TObjectPoolIdx, GlobalId>::iterator it = m_mapLid2Gid.find(id_local);
	if (it == m_mapLid2Gid.end())
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
	int idx = recieved? 1: 0;
	TRACE(TEXT("PreUpdating %s id:%d, [%E,%E,%E], [%E,%E,%E]\n"), recFlag[idx], id_local,
									curState->anyState.position.x, curState->anyState.position.y, curState->anyState.position.z,
									curState->anyState.tangent.i, curState->anyState.tangent.j, curState->anyState.tangent.k);
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
CVED::CDynObj* CExternalObjectControlImpl<TNetworkImpl>::CreatePeerDriver(CHeaderDistriParseBlock& blk, CVED::CCved* cved, cvEObjType runAs)
{
	const double cMETER_TO_FEET = 3.2808; // feet
	//
	// Get the SOL object.
	//
	const string cSolName = blk.GetSolName();

	const CSolObj* cpSolObj = cved->GetSol().GetObj( cSolName );
	if( !cpSolObj )
	{
		assert(0);
		return NULL;
	}

	//
	// Get the CVED object type and check to make sure it's valid.
	//

	//
	// Initialize the attributes.
	//
	cvTObjAttr attr = { 0 };
	attr.solId = cpSolObj->GetId();
	attr.xSize = cpSolObj->GetLength();
	attr.ySize = cpSolObj->GetWidth();
	attr.zSize = cpSolObj->GetHeight();
	attr.colorIndex = blk.GetColorIndex();
	attr.hcsmId = 0;



	double initVel = 0;
	cved->GetOwnVehicleVel(initVel);
	//
	// Get the starting location and add the vehicle's CG in the z-axis.
	//
	CPoint3D cartPos = blk.GetOwnVehPos();
	CVED::CRoadPos roadPos(*cved, cartPos);
	CVED::CDynObj* pObj = NULL;
	if( !roadPos.IsValid() )
	{
		//
		// The vehicle is currently off-road...use the offroad point.
		//

		CPoint3D targPos = roadPos.GetXYZ();
		CVector3D tan = targPos - cartPos;
		CVector3D lat( 0.0, 0.0, 0.0 );

		//
		// Create the CVED object.
		//
		pObj = cved->CreateDynObj(
							blk.GetSimName()
							, runAs
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end if vehicle offroad
	else
	{
		//
		// The vehicle is on the road.
		//
		cartPos = roadPos.GetXYZ();
        double vel;
        CVED::CRoadPos roadPosN = roadPos;
        if (initVel > 0 ){
            float distLookAhead = initVel * cMETER_TO_FEET;
            if (roadPosN.Travel(distLookAhead) == CVED::CRoadPos::eERROR){
                roadPosN = roadPos;
            }
        }
        CVector3D rotVec;
        if (initVel > 0){
            CVector3D vec1 = roadPos.GetTangentInterpolated();
            CVector3D vec2 = roadPosN.GetTangentInterpolated();
            CVector3D vec3 = vec1-vec2;

            auto targPosRp = roadPosN.GetBestXYZ();
            auto currPosRp = roadPos.GetBestXYZ();
            CVector3D targPos(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            CVector3D currPos(currPosRp.m_x,currPosRp.m_y,currPosRp.m_z);
	        CVector3D targetSeg(targPos - currPos);
            float laneWidth = 0;

            float currentMaxOffset = 6.0f;

	        //Scaling this by 1/3 the segment length
	        //was found through experimentations
	        //to work well, this should be parmaiterized
	        vec3.Scale(targetSeg.Length()/3);


            if (vec3.Length() > currentMaxOffset){
                vec3.Scale(currentMaxOffset/vec3.Length());
            }
            CPoint3D projectedXYZ  = roadPos.GetBestXYZ();
	        CVED::CRoadPos tempRoadPos(roadPos);
	        tempRoadPos.SetXYZ(projectedXYZ + vec3);
            targPosRp = tempRoadPos.GetBestXYZ();
            CVector3D vec4(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            rotVec = vec4 - currPos;
            rotVec.Normalize();
        }
		//
		// Get the staring tangent and lateral vectors.
		//
		CVector3D tan = roadPos.GetTangent();
		CVector3D lat = roadPos.GetRightVec();
        if (initVel > 0){
            double yaw = acos(rotVec.DotP(tan) );
            if (/*diff.Length()*/ yaw > 0.01){
                auto axis = tan.CrossP(rotVec);
                if (axis.m_k<0) yaw*=-1;
                tan.RotZ(yaw);
                lat.RotZ(yaw);
            }
        }
		//
		// Create the CVED object.
		//
		pObj = cved->CreateDynObj(
							blk.GetSimName()
							, runAs
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end else vehicle on the road

	//
	// Get a pointer to the vehicle object.
	//
	CVED::CVehicleObj* pVehicleObj = dynamic_cast<CVED::CVehicleObj *>( pObj );

	//
	// Set the initial velocity.
	//


	bool haveInitVel = initVel >= 0.0;
	if( haveInitVel )
	{
		// set velocity to both buffers
		pObj->SetVel( initVel, false );
	}

	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	const CSolObjVehicle* cpSolVeh =
				dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
	if( !cpSolVeh )
	{
		assert(0);
		cved->DeleteDynObj(pObj);
		return NULL;
	}

	const CDynaParams& dynaParams = cpSolVeh->GetDynaParams();

	double suspStif =
			( dynaParams.m_SuspStifMin + dynaParams.m_SuspStifMax ) / 2;
	pVehicleObj->SetSuspStif( suspStif );
	pVehicleObj->SetSuspStifImm( suspStif );
	double suspDamp =
			( dynaParams.m_SuspDampMin + dynaParams.m_SuspDampMax ) / 2;
	pVehicleObj->SetSuspDamp( suspDamp );
	pVehicleObj->SetSuspDampImm( suspDamp );

	double tireStif =
			( dynaParams.m_TireStifMin + dynaParams.m_TireStifMax ) / 2;
	pVehicleObj->SetTireStif( tireStif );
	pVehicleObj->SetTireStifImm( tireStif );
	double tireDamp =
			( dynaParams.m_TireDampMin + dynaParams.m_TireDampMax ) / 2;
	pVehicleObj->SetTireDamp( tireDamp );
	pVehicleObj->SetTireDampImm( tireDamp );
	//pVehicleObj->SetDynaFidelity( m_pI->m_dynModel );
	//pVehicleObj->SetDynaFidelityImm( m_pI->m_dynModel );
	pVehicleObj->SetQryTerrainErrCount( 0 );
	pVehicleObj->SetQryTerrainErrCountImm( 0 );
	pVehicleObj->SetDynaInitComplete( 0 );
	pVehicleObj->SetDynaInitCompleteImm( 0 );

	//
	// Set the initial audio and visual state.
	//
	pVehicleObj->SetAudioState( blk.GetAudioState() );
	pVehicleObj->SetVisualState( blk.GetVisualState() );

	// //
	// // If the audio or visual has been set then assign these values
	// // to the dials.  This way the ADO will play those sounds and
	// // display those lights until they are explicity turned off.
	// //
	// if( blk.GetVisualState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetVisualState() );
	// 	string str( buf );
	// 	m_dialVisualState.SetValue( str );
	// }

	// if( blk.GetAudioState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetAudioState() );
	// 	string str( buf );
	// 	m_dialAudioState.SetValue( str );
	// }

    //init control
    pVehicleObj->SetTargPos(cartPos);
    pVehicleObj->StoreOldTargPos();

	return pObj;

}

template<class TNetworkImpl>
bool CExternalObjectControlImpl<TNetworkImpl>::Initialize(CHeaderDistriParseBlock& hBlk, CVED::CCvedDistri* pCved)
{
	std::set<IP> localhostIps;
	GetLocalhostIps(localhostIps);

	std::list<SEG> lstDistSegs;
	std::list<IP> lstDistIps;
	int id_local = 0; //cved object id
	int numSelf = 0;
	//edo_controller, ado_controller
	cvEObjType objTypes[] = {eCV_VEHICLE, eCV_EXTERNAL_DRIVER};
	do
	{
		std::string ipStr = hBlk.GetIPV4();
		std::string ipMask = hBlk.GetIPMask();
		IP simIP = inet_addr(ipStr.c_str());
		IP simMask = inet_addr(ipMask.c_str());
		bool localhostip = (localhostIps.end() != localhostIps.find(simIP));
		if (!localhostip)
		{
			SEG seg = {simIP, simMask};
			lstDistSegs.push_back(seg);
			lstDistIps.push_back(simIP);
			CVED::CDynObj* peerObj = CreatePeerDriver(hBlk, pCved, objTypes[c_type]);
			id_local = peerObj->GetId();
			m_lstPeers.push_back(peerObj);
			GlobalId id_global = {simIP, 0};
			m_mapLid2Gid.insert(std::pair<TObjectPoolIdx, GlobalId>(id_local, id_global));
		}
		else
		{
			m_selfIp = simIP;
			hBlk.TagLocalhost();
			numSelf ++;
		}
	}while (hBlk.NextExternalBlk());

	bool ok = (numSelf == 1);
	if (ok)
	{
		InitIpclusters(lstDistSegs, m_ipClusters);
		NetworkInitialize(m_ipClusters, lstDistIps, hBlk.GetPort(), m_selfIp);
		m_pCved = pCved;
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
		if (1) //p.second.mag > 1)
			ip = p.first;
		else
			ip = p.second.ip;
		clusters.push_back(ip);
	}
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::UnInitialize()
{
	NetworkUninitialize();
	m_ipClusters.clear();
	m_mapLid2Gid.clear();
	for (std::list<CVED::CDynObj*>::iterator it = m_lstPeers.begin(); it != m_lstPeers.end(); it ++)
		m_pCved->DeleteDynObj(*it);
	m_lstPeers.clear();

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
	CDynObj* obj = m_pCved->CreateDynObj(name, eCV_VEHICLE, cAttr, cpInitPos, cpInitTran, cpInitLat);
	TObjectPoolIdx id_local = obj->GetId();
	m_mapLid2Gid[id_local] = id_global;
	m_mapGid2Ado[id_global] = obj;
}

template<class TNetworkImpl>
void CExternalObjectControlImpl<TNetworkImpl>::DeleteAdoStub(GlobalId id_global)
{
	std::map<GlobalId, CDynObj*>::iterator it = m_mapGid2Ado.find(id_global);
	ASSERT(it != m_mapGid2Ado.end());
	CDynObj* obj = (*it).second;
	TObjectPoolIdx id_local = obj->GetId();
	m_pCved->DeleteDynObj(obj);
	m_mapGid2Ado.erase(it);
	m_mapLid2Gid.erase(id_local);

}