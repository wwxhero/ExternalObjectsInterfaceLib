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
#include <matrix/vlQuaternion.h>


#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <glm/gtx/transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/euler_angles.hpp>

#include "vrlinkMath.h"

#include "PduExtObj.h"

CVrlinkDisDynamic::VrLinkConf CVrlinkDisDynamic::s_disConf = {
	DtDrDrmRvw
	, DtTimeStampRelative
	, false
	, 0.05						//translation threshold in meter
	, 0.05236					//rotation threshold in radian = 3 degree
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
	sprintf(ipStrSelf, "%u.%u.%u.%u", self[0], self[1], self[2], self[3]); //fixme: this works only for little endian structure
	sInit.setDeviceAddress(ipStrSelf);
	DtThresholder::setDfltTranslationThreshold(s_disConf.translationThreshold);
	DtThresholder::setDfltRotationThreshold(s_disConf.rotationThreshold);
	DtEntityType f18Type(s_disConf.kind, s_disConf.domain,
		s_disConf.country, s_disConf.category, s_disConf.subCategory, s_disConf.specific, s_disConf.extra);
	GlobalId id_global_self = {self, 0};
	char ipStr[16] = {0};
	for (std::list<IP>::const_iterator it = sendTo.begin()
		; it != sendTo.end()
		; it ++)
	{
		IP ip = *it;
		unsigned char* seg = (unsigned char*) &ip;
		sprintf(ipStr, "%u.%u.%u.%u", seg[0], seg[1], seg[2], seg[3]); //fixme: this works only for little endian structure
		sInit.setDestinationAddress(ipStr);
		DtExerciseConn* cnn = new DtExerciseConn(sInit, &status);
		ASSERT(DtExerciseConn::DtINIT_SUCCESS == status);
		DtEntityPublisher* pub = new DtEntityPublisher(f18Type, cnn
			, (DtDeadReckonTypes)s_disConf.drAlgor, DtForceFriendly
			, DtEntityPublisher::guiseSameAsType(), GlobalId2VrlinkId(id_global_self));
		DtEntityStateRepository* esr = pub->entityStateRep();
		DtTopoView* view = new DtTopoView(esr, s_disConf.latitude, s_disConf.longitude);
		view->setOrientation(DtTaitBryan(s_disConf.viewOriPsi, s_disConf.viewOriTheta, s_disConf.viewOriPhi));

		DtClock* clk = cnn->clock();
		clk->init();

		CnnOut& out = m_cnnsOut[ip];
		out.cnn = cnn;

		EntityPub epb = {pub, view};
		out.pubs[0] = epb;
	}

	//initialize for receiver
	DtExerciseConnInitializer rInit;
	rInit.setUseAsynchIO(true);
	rInit.setDisVersionToSend(7);
	rInit.setTimeStampType((DtTimeStampType)s_disConf.stmType);
	rInit.setDeviceAddress(ipStrSelf);

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

	for (std::map<GlobalId, EntityState>::iterator it = m_statesIn.begin()
		; it != m_statesIn.end()
		; it ++)
	{
		std::pair<GlobalId, EntityState> p = *it;
		EntityState b = p.second;
		delete b.sb;
	}

}

void CVrlinkDisDynamic::OnReceiveRawPdu( CCustomPdu* pdu, void* p)
{
	CVrlinkDisDynamic* pThis = reinterpret_cast<CVrlinkDisDynamic*>(p);
	CPduExtObj* pExtObj = static_cast<CPduExtObj*>(pdu);
	const CPduExtObj::RawState& rs = pExtObj->GetState();
	std::map<GlobalId, EntityState>::iterator it = pThis->m_statesIn.find(rs.id);
	EntityState* esb = NULL;
	if (pThis->m_statesIn.end() == it) //no state slot allocated in statesIn buffer
	{
		EntityState& state = pThis->m_statesIn[rs.id];
		state.updated = false;
		state.sb = new cvTObjStateBuf;
		memset(state.sb, 0, sizeof(cvTObjStateBuf));
		esb = &state;
	}
	else
		esb = &(*it).second;
	cvTObjState::ExternalDriverState& s = esb->sb->state.externalDriverState;
	s.visualState = rs.visualState;
	s.audioState = rs.audioState;
	s.suspStif = rs.suspStif;
	s.suspDamp = rs.suspDamp;
	s.tireStif = rs.tireStif;
	s.tireDamp = rs.tireDamp;
	s.velBrake = rs.velBrake;
	memcpy(s.posHint, rs.posHint, sizeof(rs.posHint));
	s.dynaFidelity = rs.dynaFidelity;
	esb->updated = true;
#ifdef _DEBUG
	pExtObj->printData();
#endif
}

void CVrlinkDisDynamic::Send(IP ip, GlobalId id_global, const cvTObjStateBuf& sb)
{
	EntityPublisher epb;
	bool exists_a_pub = getEntityPub(ip, id_global, epb);
	ASSERT(exists_a_pub);
	if (!exists_a_pub)
		return;
	const cvTObjState* s = (const cvTObjState*)(&sb.state);
	ExternalDriverStateTran stateTran;
	Transform(s->externalDriverState, stateTran);

#ifdef _DEBUG
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
	epb.view->setRotationalVelocity(stateTran.rot);
	epb.view->setLocation(stateTran.loc);
	epb.view->setOrientation(stateTran.ori);

	CPduExtObj pduObj(id_global, sb.state.externalDriverState);
	epb.cnn->sendStamped(pduObj);
}

bool CVrlinkDisDynamic::Receive(GlobalId id_global, const cvTObjStateBuf*& sb)
{
	std::map<GlobalId, EntityState>::iterator it = m_statesIn.find(id_global);
	if (it != m_statesIn.end())
	{
		EntityState esb = (*it).second;
		if (esb.updated)
			sb = esb.sb;
		return esb.updated;
	}
	else
		return false;
}


void CVrlinkDisDynamic::PreDynaCalc()
{
	//pre calc for sending
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
	for (std::map<GlobalId, EntityState>::iterator it = m_statesIn.begin()
		; it != m_statesIn.end()
		; it ++)
	{
		EntityState& p = (*it).second;
		p.updated = false;
	}

	m_cnnIn->clock()->setSimTime(simTime);
	m_cnnIn->drainInput();
	DtReflectedEntity* e = m_entitiesIn->first();
	for(; e != NULL; e = e->next())
	{
		const DtObjectId& id = e->entityId();
		GlobalId id_global = VrlinkId2GlobalId(id);
		unsigned char* ip = (unsigned char*)&id_global.owner;
		TRACE(TEXT("vrlink: received from [%d.%d.%d.%d]\n"), ip[0], ip[1], ip[2], ip[3]);
		std::map<GlobalId, EntityState>::iterator it = m_statesIn.find(id_global);
		EntityState* esb = NULL;
		if (m_statesIn.end() == it) //no state slot allocated in state buffer
		{
			EntityState& state = m_statesIn[id_global];
			state.updated = false;
			state.sb = new cvTObjStateBuf;
			memset(state.sb, 0, sizeof(cvTObjStateBuf));
			esb = &state;
		}
		else
		{
			esb = &(*it).second;
		}

		DtEntityStateRepository *esr = e->entityStateRep();
		esr->setAlgorithm((DtDeadReckonTypes)s_disConf.drAlgor);
		esr->useSmoother(s_disConf.smoothOn);
		DtTopoView view(esr, s_disConf.latitude, s_disConf.longitude);
		ExternalDriverStateTran stateTran;
		stateTran.vel = view.velocity();
		stateTran.acc = view.acceleration();
		stateTran.rot = view.rotationalVelocity();
		stateTran.loc = view.location();
		stateTran.ori = view.orientation();

		cvTObjStateBuf* sb = esb->sb;
		cvTObjState::ExternalDriverState* s = (cvTObjState::ExternalDriverState*)&sb->state;
		Transform(stateTran, *s);
		esb->updated = true;
	}

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
