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

inline int ID2IP(const DtObjectId& id)
{
	int ip;
	short* ip_a = (short* )&ip;
	ip_a[0] = id.site();
	ip_a[1] = id.host();
	return ip;
}

inline DtObjectId IP2ID(int ip)
{
	short* id = (short*)&ip;
	return DtObjectId(id[0], id[1], 0);
}

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

CVrlinkDisDynamic::CVrlinkDisDynamic(void)
	: m_reciver(NULL)
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
			, DtEntityPublisher::guiseSameAsType(), IP2ID(self));
		DtEntityStateRepository* esr = pub->entityStateRep();
		DtTopoView* view = new DtTopoView(esr, s_disConf.latitude, s_disConf.longitude);
		view->setOrientation(DtTaitBryan(s_disConf.viewOriPsi, s_disConf.viewOriTheta, s_disConf.viewOriPhi));

		DtClock* clk = cnn->clock();
		clk->init();

		EntityPublisher& ep = m_sendersPub[ip];
		ep.cnn = cnn;
		ep.pub = pub;
		ep.view = view;
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
	CPduExtObj::StartListening(m_reciver, OnReceiveRawPdu, this);

	for (std::list<IP>::const_iterator it = receiveFrom.begin()
		; it != receiveFrom.end()
		; it ++)
	{
		IP ip = *it;
		StateBuffer& buf = m_receivedStates[ip];
		buf.updated = false;
		buf.item = new cvTObjStateBuf;
		memset(buf.item, 0, sizeof(cvTObjStateBuf));
	}
	m_self = self;
}
void CVrlinkDisDynamic::NetworkUninitialize()
{
	for (std::map<IP, CVrlinkDisDynamic::EntityPublisher>::iterator it = m_sendersPub.begin()
		; it != m_sendersPub.end()
		; it ++)
	{
		std::pair<IP, CVrlinkDisDynamic::EntityPublisher> p = *it;
		delete p.second.view;
		delete p.second.pub;
		delete p.second.cnn;
	}
	delete m_receivedEntities;
	CPduExtObj::StopListening(m_reciver, OnReceiveRawPdu, this);
	delete m_reciver;

	for (std::map<IP, StateBuffer>::iterator it = m_receivedStates.begin()
		; it != m_receivedStates.end()
		; it ++)
	{
		std::pair<IP, StateBuffer> p = *it;
		StateBuffer b = p.second;
		delete b.item;
	}

}

void CVrlinkDisDynamic::OnReceiveRawPdu( CPduExtObj* pdu, void* p)
{
	CVrlinkDisDynamic* pThis = reinterpret_cast<CVrlinkDisDynamic*>(p);
	const CPduExtObj::ExtObjRawState& rs = pdu->GetState();
	std::map<IP, StateBuffer>::iterator it = pThis->m_receivedStates.find(rs.ip);
	if (pThis->m_receivedStates.end() == it) //rejects unrecognizable connections
		return;
	StateBuffer& sb = (*it).second;
	cvTObjState::ExternalDriverState& s = sb.item->state.externalDriverState;
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
	pdu->printData();
#endif
}

void CVrlinkDisDynamic::Send(IP ip, const cvTObjStateBuf& sb)
{
	ASSERT(m_sendersPub.find(ip) != m_sendersPub.end());
	EntityPublisher& pub = m_sendersPub[ip];
	DtExerciseConn* cnn = pub.cnn;

	const cvTObjState* s = (const cvTObjState*)(&sb.state);
	DisSend(cnn, pub, s);
	CPduExtObj pduObj(m_self, sb.state.externalDriverState);
	cnn->sendStamped(pduObj);

}
bool CVrlinkDisDynamic::Receive(IP ip, const cvTObjStateBuf*& sb)
{
	const StateBuffer& buff = m_receivedStates[ip];
	if (buff.updated)
		sb = buff.item;
	return buff.updated;
}

void CVrlinkDisDynamic::DisSend(DtExerciseConn* cnn, EntityPublisher& pub, const cvTObjState* s)
{
	ExternalDriverStateTran stateTran;
	Transform(s->externalDriverState, stateTran);
#ifdef _DEBUG
	const cvTObjState::ExternalDriverState& es = s->externalDriverState;
	glm::vec3 angularVel(stateTran.rot.x(), stateTran.rot.y(), stateTran.rot.z());
	float aL = rad2deg(glm::length(angularVel));
	glm::vec3 aDir = glm::normalize(angularVel);
	 cvTObjState::ExternalDriverState es_prime;
	Transform(stateTran, es_prime);
	TRACE(TEXT("DisSend id:%d, \n\t position: [%E,%E,%E]->[%E,%E,%E]")
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
	pub.view->setVelocity(stateTran.vel);
	pub.view->setAcceleration(stateTran.acc);
	pub.view->setRotationalVelocity(stateTran.rot);
	pub.view->setLocation(stateTran.loc);
	pub.view->setOrientation(stateTran.ori);
}

void CVrlinkDisDynamic::PreDynaCalc()
{
	//pre calc for sending
	DtTime simTime = (DtTime)m_sysClk.GetTickCnt()/(DtTime)1000; //in seconds
	for (std::map<IP, EntityPublisher>::iterator it = m_sendersPub.begin()
		; it != m_sendersPub.end()
		; it ++)
	{
		std::pair<IP, EntityPublisher> p = *it;
		DtExerciseConn* cnn = p.second.cnn;
		cnn->clock()->setSimTime(simTime);
		cnn->drainInput();
	}
	//pre calc for receiving
	for (std::map<IP, StateBuffer>::iterator it = m_receivedStates.begin()
		; it != m_receivedStates.end()
		; it ++)
	{
		StateBuffer& p = (*it).second;
		p.updated = false;
	}

	m_reciver->clock()->setSimTime(simTime);
	m_reciver->drainInput();
	DtReflectedEntity* e = m_receivedEntities->first();
	for(; e != NULL; e = e->next())
	{
		const DtObjectId& id = e->entityId();
		int ip = ID2IP(id);
		std::map<IP, StateBuffer>::iterator it = m_receivedStates.find(ip);
		if (m_receivedStates.end() == it) //rejects unrecognizable connections
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


		StateBuffer& buff = (*it).second;
		cvTObjStateBuf* sb = buff.item;
		cvTObjState::ExternalDriverState* s = (cvTObjState::ExternalDriverState*)&sb->state;
		Transform(stateTran, *s);
		buff.updated = true;
	}

}

void CVrlinkDisDynamic::PostDynaCalc()
{
	//post calc for sending
	for (std::map<IP, EntityPublisher>::iterator it = m_sendersPub.begin()
		; it != m_sendersPub.end()
		; it ++)
	{
		std::pair<IP, EntityPublisher> p = *it;
		DtEntityPublisher* pub = p.second.pub;
		pub->tick();
	}
}
