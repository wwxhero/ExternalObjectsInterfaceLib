#include "StdAfx.h"
#include "VrlinkDisDynamic.h"

#include "hcsmobject.h"
#include "cvedstrc.h"
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

#include "PduExtObj.h"

static const TVector3D c_up0 = {0, 0, 1};
static const TVector3D c_t0 = {0, -1, 0};
static const TVector3D c_l0 = {-1, 0, 0};

#if !defined ASSERT
#ifdef _DEBUG
	#define ASSERT assert
#else
	#define ASSERT(exp) void(0)
#endif
#endif



typedef struct ExternalDriverStateTran_tag
{
	DtVector32  vel;
	DtVector32  acc;
	DtVector32  rot;
	DtVector    loc;
	DtTaitBryan ori;
} ExternalDriverStateTran;

#define PI 3.1416
const double d_epsilon = FLT_EPSILON;
const float f_epsilon = 0.00000001;

inline bool Approach(double x, double t)
{
	double d = x - t;
	return d < d_epsilon && d > -d_epsilon;
}

inline bool Approach(float x, float t)
{
	float d = x - t;
	return d < f_epsilon && d > -f_epsilon;
}

template <typename T, glm::precision P>
inline bool Approach(glm::tvec3<T, P> v, T t)
{
	return Approach(v.x, t)
		&& Approach(v.y, t)
		&& Approach(v.z, t);
}





inline double deg2rad(double a)
{
	return (a/180)*PI;
}

inline double rad2deg(double r)
{
	return (r/PI)*180;
}

inline void Frame2TaitBryan(const TVector3D& tangent, const TVector3D& lateral, const TVector3D& tangent_prime, const TVector3D& lateral_prime, DtTaitBryan& ori)
{
	//todo: generate tait-bryan euler angle from [t,l,b] to [t', l', b']
	glm::dvec3 t(tangent.i, tangent.j, tangent.k);
	glm::dvec3 l(lateral.i, lateral.j, lateral.k);
	glm::dvec3 b = glm::cross(t, l);
	glm::dmat3 f;
	f[0] = t;
	f[1] = l;
	f[2] = b;

	glm::dvec3 t_prime(tangent_prime.i, tangent_prime.j, tangent_prime.k);
	glm::dvec3 l_prime(lateral_prime.i, lateral_prime.j, lateral_prime.k);
	glm::dvec3 b_prime = glm::cross(t_prime, l_prime);
	glm::dmat3 f_prime;
	f_prime[0] = t_prime;
	f_prime[1] = l_prime;
	f_prime[2] = b_prime;

	glm::dmat3 r = f_prime * glm::inverse(f);

	glm::dquat q = glm::quat_cast(r);
	DtQuaternion q_vrlink(q.w, q.x, q.y, q.z);
	ori = q_vrlink; //this call might have performance overhead
}

inline void TaitBryan2Frame(const DtTaitBryan& ori, const TVector3D& tangent, const TVector3D& lateral, TVector3D& tangent_prime, TVector3D& lateral_prime)
{
	//todo: generate tangent and lateral [t', l', b'] from orientation, and [t, l, b]
	double psi = ori.psi();
	double theta = ori.theta();
	double phi = ori.phi();
	double c1 = cos(psi);
	double s1 = sin(psi);
	double c2 = cos(theta);
	double s2 = sin(theta);
	double c3 = cos(phi);
	double s3 = sin(phi);
	glm::dmat3 r(c1*c2					,c2*s1					,-s2
				,c1*s2*s3 - c3*s1		,c1*c3 + s1*s2*s3		,c2*s3
				,s1*s3 + c1*c3*s2		,c3*s1*s2 - c1*s3		,c2*c3);
	glm::dvec3 t(tangent.i, tangent.j, tangent.k);
	glm::dvec3 t_prime = r * t;
	tangent_prime.i = t_prime.x;
	tangent_prime.j = t_prime.y;
	tangent_prime.k = t_prime.z;

	glm::dvec3 l(lateral.i, lateral.j, lateral.k);
	glm::dvec3 l_prime = r * l;
	lateral_prime.i = l_prime.x;
	lateral_prime.j = l_prime.y;
	lateral_prime.k = l_prime.z;
}

inline void AVSim2Vrlink(const TVector3D& src, DtVector32& dst, const TVector3D& tangent, const TVector3D& lateral)
{
	//src.k stands for yaw velocity in degree
	//src.j stands for pitch velocity
	//src.i stands for roll velocity
	DtTaitBryan aTB(deg2rad(src.k), deg2rad(src.j), deg2rad(src.i));
	DtQuaternion aQ(aTB);
	glm::vec3 sin_half_time_axis(aQ.x(), aQ.y(), aQ.z());
	float sin_half = glm::length(sin_half_time_axis);
	if (Approach(sin_half, 0))
	{
		dst.setX(0);
		dst.setY(0);
		dst.setZ(0);
	}
	else
	{
		float cos_half = aQ.w();
		glm::vec3 u_w = sin_half_time_axis/sin_half;
		glm::vec3 x_w(tangent.i, tangent.j, tangent.k);
		glm::vec3 y_w(lateral.i, lateral.j, lateral.k);
		glm::vec3 z_w = cross(x_w, y_w);
		glm::mat3 entity2world(x_w, y_w, z_w);
		glm::mat3 world2entity = glm::inverse(entity2world);
		glm::vec3 u_e = world2entity * u_w;
		float a_r = atan2(sin_half, cos_half) * 2;
		glm::vec3 a_time_axis = a_r * u_e;
		dst.setX(a_time_axis.x);
		dst.setY(a_time_axis.y);
		dst.setZ(a_time_axis.z);
	}
}

inline void AVVrlink2Sim(const DtVector32& src, TVector3D& dst, const TVector3D& tangent, const TVector3D& lateral)
{
	//dst.k stands for yaw velocity in degree
	//dst.j stands for pitch velocity
	//dst.i stands for roll velocity
	float a = sqrt(src.magnitudeSquared());
	if (Approach(a, 0)) //no rotation happens
	{
		dst.i = 0;
		dst.j = 0;
		dst.k = 0;
	}
	else
	{
		float inv_a = 1/a;
		glm::vec3 u_e(src.x()*inv_a, src.y()*inv_a, src.z()*inv_a);
		glm::vec3 x_w(tangent.i, tangent.j, tangent.k);
		glm::vec3 y_w(lateral.i, lateral.j, lateral.k);
		glm::vec3 z_w = cross(x_w, y_w);
		glm::mat3 entity2world(x_w, y_w, z_w);
		glm::vec3 u_w = entity2world * u_e;
		float half_a = a * 0.5f;
		float sin_half = sin(half_a);
		float cos_half = cos(half_a);
		glm::quat q(cos_half, sin_half * u_w);
		DtQuaternion q_vrlink(q.w, q.x, q.y, q.z);
		DtTaitBryan ori = q_vrlink; //this call might have performance overhead
		dst.k = rad2deg(ori.psi());
		dst.j = rad2deg(ori.theta());
		dst.i = rad2deg(ori.phi());
	}
}

inline void Transform(const cvTObjState::ExternalDriverState& src, ExternalDriverStateTran& dst, const TVector3D& tangent0, const TVector3D& lateral0)
{
	dst.loc.setX(src.position.x);
	dst.loc.setY(src.position.y);
	dst.loc.setZ(src.position.z);

	ASSERT(src.vel >= 0); //fixme: vehicle can't move backward
	double v = src.vel;
	dst.vel.setX(v * src.tangent.i);
	dst.vel.setY(v * src.tangent.j);
	dst.vel.setZ(v * src.tangent.k);

	//ASSERT(src.acc >= 0 && src.latAccel >= 0);
	double a = src.acc;
	double al = src.latAccel;
	dst.acc.setX(a * src.tangent.i + al * src.lateral.i);
	dst.acc.setY(a * src.tangent.j + al * src.lateral.j);
	dst.acc.setZ(a * src.tangent.k + al * src.lateral.k);

	AVSim2Vrlink(src.angularVel, dst.rot, src.tangent, src.lateral);

	Frame2TaitBryan(tangent0, lateral0, src.tangent, src.lateral, dst.ori);
}


inline bool Lateral(const TVector3D& up0, const TVector3D& tangentn, const DtVector32& acc, TVector3D& lateral)
{
	glm::dvec3 t(tangentn.i, tangentn.j, tangentn.k);
	glm::dvec3 a(acc.x(), acc.y(), acc.z());
	glm::dvec3 u = glm::cross(t, a);
	if (Approach<double, glm::highp>(u, 0))
		return false;
	glm::dvec3 u0(up0.i, up0.j, up0.k);
	u = glm::normalize(u);
	double proj_up0 = glm::dot(u, u0);
	if (proj_up0 < 0)
		u = -u;
	double half_theta = -PI / 4;
	glm::dquat q(cos(half_theta), sin(half_theta)*u);
	glm::dquat v(0, tangentn.i, tangentn.j, tangentn.k);
	glm::dquat q_inv(q.w, -q.x, -q.y, -q.z);
	glm::dquat v_prime = q * v * q_inv;
	lateral.i = v_prime.x;
	lateral.j = v_prime.y;
	lateral.k = v_prime.z;
	return true;
}

inline void Transform(const ExternalDriverStateTran& src, cvTObjState::ExternalDriverState& dst, const TVector3D& tangent0, const TVector3D& lateral0)
{
	dst.position.x = src.loc.x();
	dst.position.y = src.loc.y();
	dst.position.z = src.loc.z();

	DtVector32 v = src.vel;
	dst.vel = sqrt(v.magnitudeSquared());
	bool vInvalid = Approach(dst.vel, 0);
	bool aInvalid = false;
	bool forceEuler = true;
	if (!forceEuler
		&& !vInvalid)
	{
		v.normalize();
		dst.tangent.i = v.x();
		dst.tangent.j = v.y();
		dst.tangent.k = v.z();
		aInvalid = !Lateral(c_up0, dst.tangent, src.acc, dst.lateral);
	}

	if (forceEuler || vInvalid || aInvalid)
	{
		TaitBryan2Frame(src.ori, tangent0, lateral0, dst.tangent, dst.lateral);
	}

	DtVector32 a = src.acc;
	DtVector32 t = v;
	DtVector32 l(dst.lateral.i, dst.lateral.j, dst.lateral.k);

	dst.acc = a.dotProduct(t);
	double al = a.dotProduct(l);
	dst.latAccel = al;

	AVVrlink2Sim(src.rot, dst.angularVel, dst.tangent, dst.lateral);
}

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
	Transform(s->externalDriverState, stateTran, c_t0, c_l0);
#ifdef _DEBUG
	const cvTObjState::ExternalDriverState& es = s->externalDriverState;
	glm::vec3 angularVel(stateTran.rot.x(), stateTran.rot.y(), stateTran.rot.z());
	float aL = rad2deg(glm::length(angularVel));
	glm::vec3 aDir = glm::normalize(angularVel);
	 cvTObjState::ExternalDriverState es_prime;
	Transform(stateTran, es_prime, c_t0, c_l0);
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
		Transform(stateTran, *s, c_t0, c_l0);
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
