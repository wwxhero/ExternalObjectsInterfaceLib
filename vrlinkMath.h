#pragma once
#ifndef _VRLINKMATH_H
#define _VRLINKMATH_H

#include <matrix/vlQuaternion.h>

#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <glm/gtx/transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <matrix/vlVector.h>
#include <glm/glm.hpp>
#include "utility.h"
#include "objlayout.h"

#define PI 3.1416
const double d_epsilon = FLT_EPSILON;
const float f_epsilon = 0.00000001f;

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
	//generate tait-bryan euler angle from [t,l,b] to [t', l', b']
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
	//src.k stands for yaw velocity in degree:	d(yaw)/d(t)
	//src.j stands for pitch velocity:			d(pitch)/d(t)
	//src.i stands for roll velocity:			d(roll)/d(t)
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

inline void AVVrlink2SimRad(const DtVector32& src, TVector3D& dst, const TVector3D& tangent, const TVector3D& lateral)
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
		dst.k = ori.psi();
		dst.j = ori.theta();
		dst.i = ori.phi();
	}
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

static const TVector3D c_up0 = {0, 0, 1};
static const TVector3D c_t0 = {0, -1, 0};
static const TVector3D c_l0 = {-1, 0, 0};

typedef struct ExternalDriverStateTran_tag
{
	DtVector32  vel;
	DtVector32  acc;
	DtVector32  rot_v;
	DtVector    loc;
	DtTaitBryan ori;

	struct SteeringWheel
	{
		double rot;
	} steeringWheel;

	struct Tire
	{
		double rot[3];
	} tire;

} ExternalDriverStateTran;

enum {ART_SteeringWheel = 4096, ART_Tire0 = 4128, ART_Tire1 = 4160, ART_Tire2 = 4192};

typedef struct ExternalDriverStateTranLO_tag //fixme: going to be removed
{
	DtVector    loc;
	DtTaitBryan ori;
} ExternalDriverStateTranLO;

typedef struct AvatarStateTran_tag
{
	DtVector32  vel;
	DtVector32  acc;
	DtVector32  rot_v;
	DtVector    loc;
	DtTaitBryan ori;
	TAvatarJoint*     child_first;
} AvatarStateTran;

inline void Transform(const cvTObjState::ExternalDriverState& src, ExternalDriverStateTran& dst)
{
	dst.loc.setX(src.position.x);
	dst.loc.setY(src.position.y);
	dst.loc.setZ(src.position.z);

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

	AVSim2Vrlink(src.angularVel, dst.rot_v, src.tangent, src.lateral);
	Frame2TaitBryan(c_t0, c_l0, src.tangent, src.lateral, dst.ori);

	//fixme: currently VehicleState and ExternalDriverState are not compatible on the steeringWheel
	dst.steeringWheel.rot = src.steeringWheelAngle;
	dst.tire.rot[0] = src.tireRot[0];
	dst.tire.rot[1] = src.tireRot[1];
	dst.tire.rot[2] = src.tireRot[2];
}

inline void TransformLO(const cvTObjState::ExternalDriverState& src, ExternalDriverStateTranLO& dst)
{
	dst.loc.setX(src.position.x);
	dst.loc.setY(src.position.y);
	dst.loc.setZ(src.position.z);

	Frame2TaitBryan(c_t0, c_l0, src.tangent, src.lateral, dst.ori);
}

inline void Transform(const ExternalDriverStateTran& src, cvTObjState::VehicleState& a_dst)
{
	memset(&a_dst, 0, sizeof(cvTObjState::VehicleState)); //fixme: a set of vehicle attributes are ignored and being set 0
	TVehicleState& dst = a_dst.vehState;
	dst.position.x = src.loc.x();
	dst.position.y = src.loc.y();
	dst.position.z = src.loc.z();

	DtVector32 v = src.vel;
	dst.vel = sqrt(v.magnitudeSquared());

	TaitBryan2Frame(src.ori, c_t0, c_l0, dst.tangent, dst.lateral);

	DtVector32 a = src.acc;
	DtVector32 t = v;
	DtVector32 l(dst.lateral.i, dst.lateral.j, dst.lateral.k);

	dst.acc = a.dotProduct(t);
	double al = a.dotProduct(l);
	dst.latAccel = al;


	TVector3D angularVel;
	AVVrlink2SimRad(src.rot_v, angularVel, dst.tangent, dst.lateral);
	dst.rollRate = rad2deg(angularVel.i);
	dst.pitchRate = rad2deg(angularVel.j);
	dst.yawRate = rad2deg(angularVel.k);

	dst.steeringWheelAngle = src.steeringWheel.rot;
	dst.tireRot[0] = src.tire.rot[0]; 
	dst.tireRot[1] = src.tire.rot[1]; 
	dst.tireRot[2] = src.tire.rot[2];
	
}

inline void Transform(const cvTObjState::AvatarState& src, AvatarStateTran& dst)
{
	dst.loc.setX(src.position.x);
	dst.loc.setY(src.position.y);
	dst.loc.setZ(src.position.z);

	Frame2TaitBryan(c_t0, c_l0, src.tangent, src.lateral, dst.ori);
	dst.child_first = src.child_first;
}

inline void Transform(const AvatarStateTran& src, cvTObjState::AvatarState& dst)
{
	memset(&dst, 0, sizeof(cvTObjState::AvatarState));
	dst.position.x = src.loc.x();
	dst.position.y = src.loc.y();
	dst.position.z = src.loc.z();

	DtVector32 v = src.vel;
	dst.vel = sqrt(v.magnitudeSquared());

	TaitBryan2Frame(src.ori, c_t0, c_l0, dst.tangent, dst.lateral);

	DtVector32 a = src.acc;
	DtVector32 t = v;
	DtVector32 l(dst.lateral.i, dst.lateral.j, dst.lateral.k);

	dst.acc = a.dotProduct(t);
	double al = a.dotProduct(l);
	dst.latAccel = al;


	TVector3D angularVel;
	AVVrlink2SimRad(src.rot_v, angularVel, dst.tangent, dst.lateral);

	dst.child_first = src.child_first;
}

#endif