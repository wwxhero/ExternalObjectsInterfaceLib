/*****************************************************************************
 *
 * (C) Copyright 2018 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: 1.0
 *
 * Author: Mathew Wang
 *
 * Date: Feb, 6, 2018
 *
 * Description: this class implements INetworkDynamic interface with vr-link DIS9 solution
******************************************************************************/
#pragma once
#include "inetworkdynamic.h"
#include "Clock.h"
#include <map>


class DtExerciseConn;
class DtEntityPublisher;
class DtTopoView;
class DtReflectedEntityList;
union cvTObjState;
struct cvTObjStateBuf;
class CPduExtObj;

class CVrlinkDisDynamic :
	public INetworkDynamic
{
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

	typedef struct EntityPublisher_tag
	{
		DtExerciseConn* cnn;
		DtEntityPublisher* pub;
		DtTopoView* view;
	} EntityPublisher;

	typedef struct StateBuffer_tag
	{
		cvTObjStateBuf* item;
		bool updated;
	} StateBuffer;

public:
	CVrlinkDisDynamic(void);
	virtual ~CVrlinkDisDynamic(void);
	virtual void NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self);
	virtual void NetworkUninitialize();
	virtual void PreDynaCalc();
	virtual void Send(IP ip, const cvTObjStateBuf& sb);
	virtual bool Receive(IP ip, const cvTObjStateBuf*& sb);
	virtual void PostDynaCalc();
private:
	void DisSend(DtExerciseConn* cnn, EntityPublisher& pub, const cvTObjState* s);
	static void OnReceiveRawPdu( CPduExtObj* pdu, void* pThis );
private:
	std::map<IP, EntityPublisher> m_sendersPub; //currently only 1 entity will publish from client

	DtExerciseConn* m_reciver;
	DtReflectedEntityList* m_receivedEntities;
	std::map<IP, StateBuffer> m_receivedStates;

	static VrLinkConf s_disConf;
	CClockStaticAln m_sysClk;
	IP m_self;
};

