#include "StdAfx.h"
#include "VrlinkDisPedCtrl.h"
#include "cvedstrc.h"
#include <vl/exerciseConnDis.h>
#include <vl/entityPublisherDIS.h>
#include <vl/reflectedEntityList.h>
#include <vl/topoView.h>
#include <vl/reflectedEntityListDIS.h>
#include "vrlinkMath.h"
#include "PduTelePdo.h"

void CVrlinkDisPedCtrl::OnRequest4TelePdo( CCustomPdu* pdu, void* p )
{
	CPduTelePdo* pduTele = static_cast<CPduTelePdo*>(pdu);
	CVrlinkDisPedCtrl* pThis = reinterpret_cast<CVrlinkDisPedCtrl*>(p);

	CPoint3D pt;
	CVector3D t;
	CVector3D l;
	GlobalId id_global;
	pduTele->getTuple(id_global, pt, t, l);
	pThis->OnNotify_OnTelePdo(id_global, &pt, &t, &l);
}


CVrlinkDisPedCtrl::CVrlinkDisPedCtrl(void) : CVrlinkDisEdoCtrl(ped_controller)
{

}


CVrlinkDisPedCtrl::~CVrlinkDisPedCtrl(void)
{
}

void CVrlinkDisPedCtrl::NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self)
{
	CVrlinkDisEdoCtrl::NetworkInitialize(sendTo, receiveFrom, port, self);
	CCustomPdu::StartListening<CPduTelePdo, (DtPduKind)CCustomPdu::OnTelePdo>(m_cnnIn, OnRequest4TelePdo, this);
}

void CVrlinkDisPedCtrl::NetworkUninitialize()
{
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnTelePdo>(m_cnnIn, OnRequest4TelePdo, this);
	CVrlinkDisEdoCtrl::NetworkUninitialize();
}

void CVrlinkDisPedCtrl::Send(IP ip, GlobalId id_global, const cvTObjState* s)
{
	EntityPublisher epb;
	bool exists_a_pub = getEntityPub(ip, id_global, epb);
	ASSERT(exists_a_pub);
	if (!exists_a_pub)
		return;
	ExternalDriverStateTranLO stateTran;
	TransformLO(s->externalDriverState, stateTran);

	epb.view->setLocation(stateTran.loc);
	epb.view->setOrientation(stateTran.ori);

	CPduExtObj pduObj(id_global, s->externalDriverState);
	epb.cnn->sendStamped(pduObj);

	unsigned char* seg = (unsigned char*)&ip;
	TRACE(TEXT("CVrlinkDisPedCtrl: send to [%d.%d.%d.%d]\n"), seg[0], seg[1], seg[2], seg[3]);
}