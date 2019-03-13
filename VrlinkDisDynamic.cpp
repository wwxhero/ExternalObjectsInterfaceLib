#include "StdAfx.h"
#include "VrlinkDisDynamic.h"

#include "hcsmobject.h"
#include "cvedstrc.h"
#include "objlayout.h"
#include "ExternalControlImpl.h"


#include <vl/exerciseConnDis.h>
#include <vl/entityPublisherDIS.h>
#include <vl/reflectedEntityList.h>
#include <vl/topoView.h>
#include <vl/reflectedEntityListDIS.h>

#include <vl/exerciseConnInitializer.h>
#include <vlpi/entityTypes.h>

#include <vl/entityStateRepository.h>
#include <vl/reflectedEntity.h>

#include <vlpi/articulatedPart.h>

#include <queue>


#include "vrlinkMath.h"

#include "PduExtObj.h"

CVrlinkDisDynamic::VrLinkConf CVrlinkDisDynamic::s_disConf = {
	DtDrDrmRvw
	, DtTimeStampRelative
	, false
	, 0.05						//translation threshold in meter
	, 0.005236					//rotation threshold in radian = .3 degree
	, DtDeg2Rad(35.699760)		//latitude
	, DtDeg2Rad(-121.326577)	//longitude
	, 0
	, 0
	, 0
	, DtPlatform
	, DtPlatformDomainAir
	, DtUnitedStates
	, DtFighter
	, DtF18
	, 0, 0
};

CVrlinkDisDynamic::CVrlinkDisDynamic(TERMINAL type)
	: c_type(type)
	, c_ttl(10)
	, m_cnnIn(NULL)
	, m_entitiesIn(NULL)
{
}


CVrlinkDisDynamic::~CVrlinkDisDynamic(void)
{
}


void CVrlinkDisDynamic::NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self)
{
	m_sysClk.StartClock();

	DtExerciseConn::InitializationStatus status = DtExerciseConn::DtINIT_SUCCESS;
	//initialize for sender
	DtExerciseConnInitializer sInit;
	sInit.setUseAsynchIO(true);
	sInit.setDisVersionToSend(7);
	sInit.setTimeStampType((DtTimeStampType)s_disConf.stmType);
	char ipStrSelf[16] = {0};
	unsigned char* seg = (unsigned char*) &self;
	sprintf(ipStrSelf, "%u.%u.%u.%u", seg[0], seg[1], seg[2], seg[3]); //fixme: this works only for little endian structure
	sInit.setDeviceAddress(ipStrSelf);
	DtThresholder::setDfltTranslationThreshold(s_disConf.translationThreshold);
	DtThresholder::setDfltRotationThreshold(s_disConf.rotationThreshold);
	std::vector<DtString> mcAddrStr;
	std::vector<IP>		mcAddrIp;
	char ipStr[16] = {0};
	std::list<IP>::const_iterator it;
	for (it = sendTo.begin()
		; it != sendTo.end()
		; it ++)
	{
		IP ip = *it;
		unsigned char* seg = (unsigned char*) &ip;
		sprintf(ipStr, "%u.%u.%u.%u", seg[0], seg[1], seg[2], seg[3]); //fixme: this works only for little endian structure
		mcAddrStr.push_back((DtString)ipStr);
		mcAddrIp.push_back(ip);
	}

	sInit.setMulticastAddresses(mcAddrStr);
	sInit.setMulticastTtl(c_ttl);

	for (std::size_t i = 0; i < mcAddrStr.size(); i ++)
	{
		DtString ipDst(mcAddrStr[i]);
		sInit.setDestinationAddress(ipDst);

		DtExerciseConn* cnn = new DtExerciseConn(sInit, &status);
		ASSERT(DtExerciseConn::DtINIT_SUCCESS == status);
		CnnOut& out = m_cnnsOut[mcAddrIp[i]];
		out.cnn = cnn;

		DtClock* clk = cnn->clock();
		clk->init();
	}

	//initialize for receiver
	DtExerciseConnInitializer rInit;
	rInit.setUseAsynchIO(true);
	rInit.setDisVersionToSend(7);
	rInit.setTimeStampType((DtTimeStampType)s_disConf.stmType);
	rInit.setDeviceAddress(ipStrSelf);

	rInit.setMulticastAddresses(mcAddrStr);
	rInit.setMulticastTtl(c_ttl);

	m_cnnIn = new DtExerciseConn(rInit, &status);
	ASSERT(DtExerciseConn::DtINIT_SUCCESS == status);
	m_entitiesIn = new DtReflectedEntityList(m_cnnIn);
	DtClock* clk = m_cnnIn->clock();
	clk->init();
	CCustomPdu::StartListening<CPduExtObj, (DtPduKind)CCustomPdu::ExtObjState>(m_cnnIn, OnReceiveRawPdu, this);

	m_self = self;
}

void CVrlinkDisDynamic::NetworkUninitialize()
{
	for (std::map<IP, CnnOut>::iterator it_OutCnn = m_cnnsOut.begin()
		; it_OutCnn != m_cnnsOut.end()
		; it_OutCnn ++)
	{
		std::pair<IP, CnnOut> p = *it_OutCnn;
		DtExerciseConn* cnn = p.second.cnn;
		for (std::map<TObjectPoolIdx, EntityPub>::iterator it_pub = p.second.pubs.begin()
			; it_pub != p.second.pubs.end()
			; it_pub ++)
		{
			EntityPub epb = (*it_pub).second;
			delete epb.view;
			delete epb.pub;
		}
		delete cnn;
	}

	delete m_entitiesIn;
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::ExtObjState>(m_cnnIn, OnReceiveRawPdu, this);
	delete m_cnnIn;

}

void CVrlinkDisDynamic::OnReceiveRawPdu( CCustomPdu* pdu, void* p)
{
	CVrlinkDisDynamic* pThis = reinterpret_cast<CVrlinkDisDynamic*>(p);
	CPduExtObj* pExtObj = static_cast<CPduExtObj*>(pdu);
	const CPduExtObj::RawState& rs = pExtObj->GetState();
	pThis->m_rsCached[rs.id] = rs;
#ifdef _DEBUG
	pExtObj->printData();
#endif
}
void CVrlinkDisDynamic::SendArt(IP ip, GlobalId id_global, const cvTObjState* s)
{
	EntityPublisher epb;
	bool exists_a_pub = getEntityPub(ip, id_global, epb);
	ASSERT(exists_a_pub);
	if (!exists_a_pub)
		return;
	AvatarStateTran stateTran;
	Transform(s->avatarState, stateTran);

#ifdef _VERIFY_TRANSFORM
	const cvTObjState::AvatarState& es = s->externalDriverState;
	glm::vec3 angularVel(stateTran.rot.x(), stateTran.rot.y(), stateTran.rot.z());
	float aL = rad2deg(glm::length(angularVel));
	glm::vec3 aDir = glm::normalize(angularVel);
	cvTObjState::AvatarState es_prime;
	Transform(stateTran, es_prime);
	TRACE(TEXT("Send id:%d, \n\t position: [%E,%E,%E]->[%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]->[%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]->[%E,%E,%E]")
							TEXT(", \n\t vel: [%E]->[%E]")
							TEXT(", \n\t acc: [%E]->[%E]")
							TEXT(", \n\t latAccel: [%E]->[%E]")
							TEXT(", \n\t angularVel: [%E, %E, %E]->[%E, %E, %E] = [%E]*<%E, %E, %E>\n")
							TEXT(", \n\t euler: [%E, %E, %E]\n")
				, 0
				, es.position.x, es.position.y, es.position.z, es_prime.position.x, es_prime.position.y, es_prime.position.z
				, es.tangent.i, es.tangent.j, es.tangent.k, es_prime.tangent.i, es_prime.tangent.j, es_prime.tangent.k
				, es.lateral.i, es.lateral.j, es.lateral.k, es_prime.lateral.i, es_prime.lateral.j, es_prime.lateral.k
				, es.vel, es_prime.vel
				, es.acc, es_prime.acc
				, es.latAccel, es_prime.latAccel
				, es.angularVel.i, es.angularVel.j, es.angularVel.k, es_prime.angularVel.i, es_prime.angularVel.j, es_prime.angularVel.k, aL, aDir.x, aDir.y, aDir.z
				, stateTran.ori.phi(), stateTran.ori.theta(), stateTran.ori.psi());
#endif
	epb.view->setVelocity(stateTran.vel);
	epb.view->setAcceleration(stateTran.acc);
	epb.view->setRotationalVelocity(stateTran.rot_v);
	epb.view->setLocation(stateTran.loc);
	epb.view->setOrientation(stateTran.ori);
	//traverse the joint angle tree: for sending the articulated structure joints
	DtEntityStateRepository* entity = epb.pub->esr();
	DtArticulatedPartCollection* artPartCol = entity->artPartList();
#ifdef _DEBUG
	unsigned int n_params = artPartCol->totalParameterCount();
	unsigned int n_parts = artPartCol->partCount();
#endif
	TAvatarJoint root = VIRTUAL_ROOT(stateTran.child_first);
	std::queue<TAvatarJoint*> bft_queue;
	bft_queue.push(&root);
	while (!bft_queue.empty())
	{
		TAvatarJoint* j_p = bft_queue.front();
		DtArticulatedPart& art_p = artPartCol->getPart(j_p->type);
		bft_queue.pop();
		TAvatarJoint* j_c = j_p->child_first;
		while (NULL != j_c)
		{
			bft_queue.push(j_c);
			DtArticulatedPart& art_c = artPartCol->getPart(j_c->type);
			art_c.setParameter(DtApAzimuth, j_c->angle.k);
			//art_c.setParameter(DtApAzimuthRate, 0);
			art_c.setParameter(DtApElevation, j_c->angle.j);
			//art_c.setParameter(DtApElevationRate, 0);
			art_c.setParameter(DtApRotation, j_c->angle.i);
			//art_c.setParameter(DtApRotationRate, 0);
			art_c.setParameter(DtApX, j_c->offset.i);
			//art_c.setParameter(DtApXRate, 0);
			art_c.setParameter(DtApY, j_c->offset.j);
			//art_c.setParameter(DtApYRate, 0);
			art_c.setParameter(DtApZ, j_c->offset.k);
			//art_c.setParameter(DtApZRate, 0);
			j_c = j_c->sibling_next;
			bool attached = artPartCol->attachPart(&art_c, &art_p);
#ifdef _DEBUG
			if (!attached)
				TRACE(TEXT("\nerror: attaching %s <== %s failed"), j_p->name, j_c->name);
#endif
		}
	}
#ifdef _DEBUG
	unsigned int n_params_prime = artPartCol->totalParameterCount();
	unsigned int n_parts_prime = artPartCol->partCount();
	TRACE(TEXT("\narticulated params:%d=>%d parts:%d=>%d\n"), n_params, n_params_prime, n_parts, n_parts_prime);
#endif

	//CPduExtObj pduObj(id_global, sb.state.externalDriverState);
	//epb.cnn->sendStamped(pduObj);

	unsigned char* seg = (unsigned char*)&ip;
	TRACE(TEXT("vrlink: sendArt to [%d.%d.%d.%d]\n"), seg[0], seg[1], seg[2], seg[3]);
}

void CVrlinkDisDynamic::Send(IP ip, GlobalId id_global, const cvTObjState* s)
{
	EntityPublisher epb;
	bool exists_a_pub = getEntityPub(ip, id_global, epb);
	ASSERT(exists_a_pub);
	if (!exists_a_pub)
		return;
	ExternalDriverStateTran stateTran;
	Transform(s->externalDriverState, stateTran);

#ifdef _VERIFY_TRANSFORM
	const cvTObjState::ExternalDriverState& es = s->externalDriverState;
	glm::vec3 angularVel(stateTran.rot.x(), stateTran.rot.y(), stateTran.rot.z());
	float aL = rad2deg(glm::length(angularVel));
	glm::vec3 aDir = glm::normalize(angularVel);
	cvTObjState::ExternalDriverState es_prime;
	Transform(stateTran, es_prime);
	TRACE(TEXT("Send id:%d, \n\t position: [%E,%E,%E]->[%E,%E,%E]")
							TEXT(", \n\t tangent: [%E,%E,%E]->[%E,%E,%E]")
							TEXT(", \n\t lateral: [%E,%E,%E]->[%E,%E,%E]")
							TEXT(", \n\t vel: [%E]->[%E]")
							TEXT(", \n\t acc: [%E]->[%E]")
							TEXT(", \n\t latAccel: [%E]->[%E]")
							TEXT(", \n\t angularVel: [%E, %E, %E]->[%E, %E, %E] = [%E]*<%E, %E, %E>\n")
							TEXT(", \n\t euler: [%E, %E, %E]\n")
				, 0
				, es.position.x, es.position.y, es.position.z, es_prime.position.x, es_prime.position.y, es_prime.position.z
				, es.tangent.i, es.tangent.j, es.tangent.k, es_prime.tangent.i, es_prime.tangent.j, es_prime.tangent.k
				, es.lateral.i, es.lateral.j, es.lateral.k, es_prime.lateral.i, es_prime.lateral.j, es_prime.lateral.k
				, es.vel, es_prime.vel
				, es.acc, es_prime.acc
				, es.latAccel, es_prime.latAccel
				, es.angularVel.i, es.angularVel.j, es.angularVel.k, es_prime.angularVel.i, es_prime.angularVel.j, es_prime.angularVel.k, aL, aDir.x, aDir.y, aDir.z
				, stateTran.ori.phi(), stateTran.ori.theta(), stateTran.ori.psi());
#endif
	epb.view->setVelocity(stateTran.vel);
	epb.view->setAcceleration(stateTran.acc);
	epb.view->setRotationalVelocity(stateTran.rot_v);
	epb.view->setLocation(stateTran.loc);
	epb.view->setOrientation(stateTran.ori);

	//sending out steering wheel angles and tire rotation
	DtEntityStateRepository* entity = epb.pub->esr();
	DtArticulatedPartCollection* artPartCol = entity->artPartList();
	DtArticulatedPart& art_sw = artPartCol->getPart(ART_SteeringWheel);
	art_sw.setParameter(DtApRotation, stateTran.steeringWheel.rot);
	unsigned int tireId[] = { ART_Tire0, ART_Tire1, ART_Tire2 };
	float rot_v[] = { stateTran.tire.rot[0], stateTran.tire.rot[1], stateTran.tire.rot[2] };
	for (int i_tire = 0; i_tire < sizeof(tireId) / sizeof(unsigned int); i_tire++)
	{
		DtArticulatedPart& art_tire = artPartCol->getPart(tireId[i_tire]);
		art_tire.setParameter(DtApRotation, rot_v[i_tire]);
	}
	

	//CPduExtObj pduObj(id_global, sb.state.externalDriverState);
	//epb.cnn->sendStamped(pduObj);

	unsigned char* seg = (unsigned char*)&ip;
	TRACE(TEXT("vrlink: send to [%d.%d.%d.%d]\n"), seg[0], seg[1], seg[2], seg[3]);
}

bool CVrlinkDisDynamic::Receive(GlobalId id_global, cvTObjState* state)
{
	DtObjectId eid = GlobalId2VrlinkId(id_global);
	DtReflectedEntity* e = m_entitiesIn->lookup(eid);
	bool received = (NULL != e);
	if (received)
	{
		unsigned char* ip = (unsigned char*)&id_global.owner;
		TRACE(TEXT("vrlink: received from [%d.%d.%d.%d]\n"), ip[0], ip[1], ip[2], ip[3]);

		DtEntityStateRepository *esr = e->entityStateRep();
		esr->setAlgorithm((DtDeadReckonTypes)s_disConf.drAlgor);
		esr->useSmoother(s_disConf.smoothOn);
		DtTopoView view(esr, s_disConf.latitude, s_disConf.longitude);
		ExternalDriverStateTran stateTran;
		stateTran.vel = view.velocity();
		stateTran.acc = view.acceleration();
		stateTran.rot_v = view.rotationalVelocity();
		stateTran.loc = view.location();
		stateTran.ori = view.orientation();

		DtArticulatedPartCollection* artPartCol = esr->artPartList();
		DtArticulatedPart& art_sw = artPartCol->getPart(ART_SteeringWheel);
		stateTran.steeringWheel.rot = art_sw.getParameterValue(DtApRotation);
		unsigned int tire_id[] = { ART_Tire0, ART_Tire1, ART_Tire2 };
		double* tire_rot = stateTran.tire.rot;
		for (int i_tire = 0; i_tire < sizeof(tire_id) / sizeof(unsigned int); i_tire++)
		{
			DtArticulatedPart& art_tire = artPartCol->getPart(tire_id[i_tire]);
			float rot = art_tire.getParameterValue(DtApRotation);
			tire_rot[i_tire] = rot;
		}

		cvTObjState::VehicleState* s = (cvTObjState::VehicleState*)&state->vehicleState;
		Transform(stateTran, *s);

		// cache raw state into rs.id slot
		std::map<GlobalId, CPduExtObj::RawState>::iterator it = m_rsCached.find(id_global);
		if (m_rsCached.end() != it)
		{
			const CPduExtObj::RawState& rs = it->second;
			s->vehState.visualState = rs.visualState;
			s->vehState.audioState = rs.audioState;
			s->vehState.suspStif = rs.suspStif;
			s->vehState.suspDamp = rs.suspDamp;
			s->vehState.tireStif = rs.tireStif;
			s->vehState.tireDamp = rs.tireDamp;
			s->vehState.velBrake = rs.velBrake;
			memcpy(s->vehState.posHint, rs.posHint, sizeof(rs.posHint));
			s->vehState.dynaFidelity = rs.dynaFidelity;
			unsigned char* seg = (unsigned char*)&rs.id.owner;
			TRACE(TEXT("vrlink:raw pdu received from %d.%d.%d.%d:[%d]\n"), seg[0], seg[1], seg[2], seg[3], rs.id.objId);
		}

	}
	return received;
}

bool CVrlinkDisDynamic::ReceiveArt(GlobalId id_global, cvTObjState* s)
{
	DtObjectId eid = GlobalId2VrlinkId(id_global);
	DtReflectedEntity* e = m_entitiesIn->lookup(eid);
	bool received = (NULL != e);
	if (received)
	{
		unsigned char* ip = (unsigned char*)&id_global.owner;
		TRACE(TEXT("vrlink: received from [%d.%d.%d.%d]\n"), ip[0], ip[1], ip[2], ip[3]);

		DtEntityStateRepository *esr = e->entityStateRep();
		assert(NULL != esr);
		esr->setAlgorithm((DtDeadReckonTypes)s_disConf.drAlgor);
		esr->useSmoother(s_disConf.smoothOn);
		DtTopoView view(esr, s_disConf.latitude, s_disConf.longitude);
		AvatarStateTran stateTran;
		cvTObjState::AvatarState* s_a = (cvTObjState::AvatarState*)&s->avatarState;
		stateTran.vel = view.velocity();
		stateTran.acc = view.acceleration();
		stateTran.rot_v = view.rotationalVelocity();
		stateTran.loc = view.location();
		stateTran.ori = view.orientation();
		stateTran.child_first = s_a->child_first;

		//traverse the joint angle tree: for reading the articulated structure joints
		DtArticulatedPartCollection* artPartCol = esr->artPartList();
		TAvatarJoint root = VIRTUAL_ROOT(stateTran.child_first);
		std::queue<TAvatarJoint*> bft_queue;
		bft_queue.push(&root);
		while (!bft_queue.empty())
		{
			TAvatarJoint* j_p = bft_queue.front();
			DtArticulatedPart& art_p = artPartCol->getPart(j_p->type);
			bft_queue.pop();
			TAvatarJoint* j_c = j_p->child_first;
			while (NULL != j_c)
			{
				bft_queue.push(j_c);
				DtArticulatedPart& art_c = artPartCol->getPart(j_c->type);
				j_c->angle.k = art_c.getParameterValue(DtApAzimuth);
				//j_c->angleRate.k = art_c.getParameterValue(DtApAzimuthRate);
				j_c->angle.j = art_c.getParameterValue(DtApElevation);
				//j_c->angleRate.j = art_c.getParameterValue(DtApElevationRate);
				j_c->angle.i = art_c.getParameterValue(DtApRotation);
				//j_c->angleRate.i = art_c.getParameterValue(DtApRotationRate);
				j_c->offset.i = art_c.getParameterValue(DtApX);
				//j_c->offsetRate.i = art_c.getParameterValue(DtApXRate);
				j_c->offset.j = art_c.getParameterValue(DtApY);
				//j_c->offsetRate.j = art_c.getParameterValue(DtApYRate);
				j_c->offset.k = art_c.getParameterValue(DtApZ);
				//j_c->offsetRate.k = art_c.getParameterValue(DtApZRate);
				bool attached = artPartCol->attachPart(&art_c, &art_p);
#ifdef _DEBUG
				if (!attached)
					TRACE(TEXT("\nerror: attaching %s <== %s failed"), j_p->name, j_c->name);
#endif
				j_c = j_c->sibling_next;
			}
		}


		Transform(stateTran, *s_a);
	}
	return received;
}


void CVrlinkDisDynamic::PreDynaCalc()
{
	//pre calc for sending
	m_rsCached.clear();
	DtTime simTime = (DtTime)m_sysClk.GetTickCnt()/(DtTime)1000; //in seconds
	for (std::map<IP, CnnOut>::iterator it = m_cnnsOut.begin()
		; it != m_cnnsOut.end()
		; it ++)
	{
		std::pair<IP, CnnOut> p = *it;
		DtExerciseConn* cnn = p.second.cnn;
		cnn->clock()->setSimTime(simTime);
		cnn->drainInput();
	}
	//pre calc for receiving
	m_cnnIn->clock()->setSimTime(simTime);
	m_cnnIn->drainInput();
}

void CVrlinkDisDynamic::PostDynaCalc()
{
	//post calc for sending
	for (std::map<IP, CnnOut>::iterator it = m_cnnsOut.begin()
		; it != m_cnnsOut.end()
		; it ++)
	{
		std::pair<IP, CnnOut> p = *it;
		for (std::map<TObjectPoolIdx, EntityPub>::iterator it_pubs = p.second.pubs.begin()
			; it_pubs != p.second.pubs.end()
			; it_pubs ++)
		{
			DtEntityPublisher* pub = (*it_pubs).second.pub;
			pub->tick();
		}
	}
}
