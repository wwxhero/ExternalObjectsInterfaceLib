/*****************************************************************************
 *
 * (C) Copyright 2018 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: 1.0
 *
 * Author: Matthew Wang
 *
 * Date: Feb, 6, 2018
 *
 * Description: this class implements INetworkDynamic interface with vr-link DIS9 solution
******************************************************************************/
#pragma once
#include <map>
#include <set>
#include "inetworkdynamic.h"
#include "Clock.h"
#include "LibExternalObjectIfNetwork.h"
#include "PduExtObj.h"

class DtExerciseConn;
class DtEntityPublisher;
class DtTopoView;
class DtReflectedEntityList;


typedef struct JointId_tag
{
	IP owner;
	TObjectPoolIdx objId;
	int partType;			//DtArtPartType
} JointId;

inline bool operator < (JointId id1, JointId id2)
{
	return id1.owner < id2.owner
		|| (id1.owner == id2.owner && id1.objId < id2.objId)
		|| (id1.owner == id2.owner && id1.objId == id2.objId && id1.partType < id2.partType);
}

class CVrlinkDisDynamic :
	public INetworkDynamic
{
protected:
	typedef struct VrLinkConf_tag
	{
		int drAlgor;
		int stmType;
		bool smoothOn;

		double translationThreshold;
		double rotationThreshold;

		double latitude;
		double longitude;
		double viewOriPsi;
		double viewOriTheta;
		double viewOriPhi;

		//entity type
		int kind;
		int domain;
		int country;
		int category;

        int subCategory;
        int specific;
        int extra;
	} VrLinkConf;

	typedef struct EntityPub_tag
	{
		DtEntityPublisher* pub;
		DtTopoView* view;
	} EntityPub;


	typedef struct CnnOut_tag
	{
		DtExerciseConn* cnn;
		std::map<TObjectPoolIdx, EntityPub> pubs;
	} CnnOut;

	typedef struct EntityPublisher_tag
	{
		DtExerciseConn* cnn;
		DtEntityPublisher* pub;
		DtTopoView* view;
	} EntityPublisher;

public:
	CVrlinkDisDynamic(TERMINAL type);
	virtual ~CVrlinkDisDynamic(void);
	virtual void NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self);
	virtual void NetworkUninitialize();
	virtual void PreDynaCalc();
	virtual void Send(IP ip, GlobalId id_global, const cvTObjState* s);
	virtual bool Receive(GlobalId id_global, cvTObjState* s);
	virtual void SendArt(IP ip, GlobalId id_global, const cvTObjState* s); //fixme: move it to VrlinkDisPedCtrl
	virtual void SendArt(IP ip, GlobalId id_global, GlobalId id_parent, const cvTObjState* s); //fixme: move it to VrlinkDisPedCtrl
	virtual bool ReceiveArt(GlobalId id_global, cvTObjState* s);
	virtual bool ReceiveArt(GlobalId id_global, GlobalId id_parent, cvTObjState* s);
	virtual void PostDynaCalc();
protected:
	inline bool getEntityPub(IP ip, GlobalId id_global, EntityPublisher& pub)
	{
		TObjectPoolIdx objId = id_global.objId;
		std::map<IP, CnnOut>::iterator it_cnnOut;
		std::map<TObjectPoolIdx, EntityPub>::iterator it_pub;
		if (m_cnnsOut.end() == (it_cnnOut = m_cnnsOut.find(ip))
			||  (*it_cnnOut).second.pubs.end() == (it_pub = (*it_cnnOut).second.pubs.find(objId)))
			return false;
		else
		{
			pub.cnn = (*it_cnnOut).second.cnn;
			pub.pub = (*it_pub).second.pub;
			pub.view = (*it_pub).second.view;
			return true;
		}
	}
	static void OnReceiveRawPdu( CCustomPdu* pdu, void* pThis );
protected:
	inline GlobalId VrlinkId2GlobalId(const DtObjectId& id)
	{
		GlobalId id_global;
		short* ip_a = (short* )&id_global.owner;
		ip_a[0] = id.site();
		ip_a[1] = id.host();
		id_global.objId = id.entityNum();
		return id_global;
	}

	inline DtObjectId GlobalId2VrlinkId(GlobalId id_global)
	{
		short* id = (short*)&id_global.owner;
		return DtObjectId(id[0], id[1], id_global.objId);
	}
protected:
	std::map<IP, CnnOut> m_cnnsOut;
	std::map<GlobalId, CPduExtObj::RawState> m_rsCached;
	DtExerciseConn* m_cnnIn;
	DtReflectedEntityList* m_entitiesIn;

	static VrLinkConf s_disConf;
	CClockStaticAln m_sysClk;
	IP m_self;


public:
	const TERMINAL c_type;
	const int c_ttl;

};

