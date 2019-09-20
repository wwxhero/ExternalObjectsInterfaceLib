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

