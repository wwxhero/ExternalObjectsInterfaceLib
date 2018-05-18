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
	, m_reciver(NULL)
	, m_receivedEntities(NULL)
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
	DtThresholder::setDfltTranslationThreshold(s_disConf.translationThreshold);
	DtThresholder::setDfltRotationThreshold(s_disConf.rotationThreshold);
	DtEntityType f18Type(s_disConf.kind, s_disConf.domain,
		s_disConf.country, s_disConf.category, s_disConf.subCategory, s_disConf.specific, s_disConf.extra);
	GlobalId id_global_proxy = {self, 0};
	char ipStr[16] = {0};
	for (std::list<IP>::const_iterator it = sendTo.begin()
		; it != sendTo.end()
		; it ++)
	{
		IP ip = *it;
		unsigned char* seg = (unsigned char*) &ip;
		sprintf(ipStr, "%u.%u.%u.%u", seg[0], seg[1], seg[2], seg[3]);//fixme: this works only for little endian structure
		sInit.setDestinationAddress(ipStr);
		DtExerciseConn* cnn = new DtExerciseConn(sInit, &status);
		ASSERT(DtExerciseConn::DtINIT_SUCCESS == status);
		DtEntityPublisher* pub = new DtEntityPublisher(f18Type, cnn
			, (DtDeadReckonTypes)s_disConf.drAlgor, DtForceFriendly
			, DtEntityPublisher::guiseSameAsType(), GlobalId2VrlinkId(id_global_proxy));
		DtEntityStateRepository* esr = pub->entityStateRep();
		DtTopoView* view = new DtTopoView(esr, s_disConf.latitude, s_disConf.longitude);
		view->setOrientation(DtTaitBryan(s_disConf.viewOriPsi, s_disConf.viewOriTheta, s_disConf.viewOriPhi));

		DtClock* clk = cnn->clock();
		clk->init();

		ProxyCnn& ep = m_proxyCnns[ip];
		ep.cnn = cnn;

		EntityProxy proxy = {pub, view};
		ep.pubs[0] = proxy;
	}

	//initialize for receiver
	DtExerciseConnInitializer rInit;
	rInit.setUseAsynchIO(true);
	rInit.setDisVersionToSend(7);
	rInit.setTimeStampType((DtTimeStampType)s_disConf.stmType);
	unsigned char* seg = (unsigned char*) &self;
	sprintf(ipStr, "%u.%u.%u.%u", seg[0], seg[1], seg[2], seg[3]); //fixme: this works only for little endian structure
	rInit.setDeviceAddress(ipStr);

	m_reciver = new DtExerciseConn(rInit, &status);
	ASSERT(DtExerciseConn::DtINIT_SUCCESS == status);
	m_receivedEntities = new DtReflectedEntityList(m_reciver);
	DtClock* clk = m_reciver->clock();
	clk->init();
	CCustomPdu::StartListening<CPduExtObj, (DtPduKind)CCustomPdu::ExtObjState>(m_reciver, OnReceiveRawPdu, this);

	for (std::list<IP>::const_iterator it = receiveFrom.begin()
		; it != receiveFrom.end()
		; it ++)
	{
		IP ip = *it;
		GlobalId id_global_stub = {ip, 0};
		EntityStub& buf = m_reciversStub[id_global_stub];
		buf.updated = false;
		buf.sb = new cvTObjStateBuf;
		memset(buf.sb, 0, sizeof(cvTObjStateBuf));
	}
	m_self = self;
}

void CVrlinkDisDynamic::NetworkUninitialize()
{
	for (std::map<IP, ProxyCnn>::iterator it_proxyCnn = m_proxyCnns.begin()
		; it_proxyCnn != m_proxyCnns.end()
		; it_proxyCnn ++)
	{
		std::pair<IP, ProxyCnn> p = *it_proxyCnn;
		DtExerciseConn* cnn = p.second.cnn;
		for (std::map<TObjectPoolIdx, EntityProxy>::iterator it_pub = p.second.pubs.begin()
			; it_pub != p.second.pubs.end()
			; it_pub ++)
		{
			EntityProxy proxyEntity = (*it_pub).second;
			delete proxyEntity.view;
			delete proxyEntity.pub;
		}
		delete cnn;
	}

	delete m_receivedEntities;
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::ExtObjState>(m_reciver, OnReceiveRawPdu, this);
	delete m_reciver;

	for (std::map<GlobalId, EntityStub>::iterator it = m_reciversStub.begin()
		; it != m_reciversStub.end()
		; it ++)
	{
		std::pair<GlobalId, EntityStub> p = *it;
		EntityStub b = p.second;
		delete b.sb;
	}

}

void CVrlinkDisDynamic::OnReceiveRawPdu( CCustomPdu* pdu, void* p)
{
	CVrlinkDisDynamic* pThis = reinterpret_cast<CVrlinkDisDynamic*>(p);
	CPduExtObj* pExtObj = static_cast<CPduExtObj*>(pdu);
	const CPduExtObj::RawState& rs = pExtObj->GetState();
	std::map<GlobalId, EntityStub>::iterator it = pThis->m_reciversStub.find(rs.id);
	if (pThis->m_reciversStub.end() == it) //rejects unrecognizable connections
		return;
	EntityStub& esb = (*it).second;
	cvTObjState::ExternalDriverState& s = esb.sb->state.externalDriverState;
	s.visualState = rs.visualState;
	s.audioState = rs.audioState;
	s.suspStif = rs.suspStif;
	s.suspDamp = rs.suspDamp;
	s.tireStif = rs.tireStif;
	s.tireDamp = rs.tireDamp;
	s.velBrake = rs.velBrake;
	memcpy(s.posHint, rs.posHint, sizeof(rs.posHint));
	s.dynaFidelity = rs.dynaFidelity;
#ifdef _DEBUG
	pExtObj->printData();
#endif
}

void CVrlinkDisDynamic::Send(IP ip, GlobalId id_global, const cvTObjStateBuf& sb)
{
	EntityPublisher epb;
	bool exists_a_pub = EntityPub(ip, id_global, epb);
	ASSERT(exists_a_pub);
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
	std::map<GlobalId, EntityStub>::iterator it = m_reciversStub.find(id_global);
	if (it != m_reciversStub.end())
	{
		EntityStub esb = (*it).second;
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
	for (std::map<IP, ProxyCnn>::iterator it = m_proxyCnns.begin()
		; it != m_proxyCnns.end()
		; it ++)
	{
		std::pair<IP, ProxyCnn> p = *it;
		DtExerciseConn* cnn = p.second.cnn;
		cnn->clock()->setSimTime(simTime);
		cnn->drainInput();
	}
	//pre calc for receiving
	for (std::map<GlobalId, EntityStub>::iterator it = m_reciversStub.begin()
		; it != m_reciversStub.end()
		; it ++)
	{
		EntityStub& p = (*it).second;
		p.updated = false;
	}

	m_reciver->clock()->setSimTime(simTime);
	m_reciver->drainInput();
	DtReflectedEntity* e = m_receivedEntities->first();
	for(; e != NULL; e = e->next())
	{
		const DtObjectId& id = e->entityId();
		std::map<GlobalId, EntityStub>::iterator it = m_reciversStub.find(VrlinkId2GlobalId(id));
		if (m_reciversStub.end() == it) //rejects unrecognizable connections
			continue;

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


		EntityStub& esb = (*it).second;
		cvTObjStateBuf* sb = esb.sb;
		cvTObjState::ExternalDriverState* s = (cvTObjState::ExternalDriverState*)&sb->state;
		Transform(stateTran, *s);
		esb.updated = true;
	}

}

void CVrlinkDisDynamic::PostDynaCalc()
{
	//post calc for sending
	for (std::map<IP, ProxyCnn>::iterator it = m_proxyCnns.begin()
		; it != m_proxyCnns.end()
		; it ++)
	{
		std::pair<IP, ProxyCnn> p = *it;
		for (std::map<TObjectPoolIdx, EntityProxy>::iterator it_pubs = p.second.pubs.begin()
			; it_pubs != p.second.pubs.end()
			; it_pubs ++)
		{
			DtEntityPublisher* pub = (*it_pubs).second.pub;
			pub->tick();
		}
	}
}
