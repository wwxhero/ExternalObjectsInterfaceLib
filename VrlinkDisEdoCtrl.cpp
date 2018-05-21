#include "StdAfx.h"
#include "VrlinkDisEdoCtrl.h"
#include "PduCrtAdo.h"
#include "PduDelAdo.h"


void CVrlinkDisEdoCtrl::OnRequest4CreateAdo( CCustomPdu* pdu, void* p )
{
	CPduCrtAdo* pduCrtAdo = static_cast<CPduCrtAdo*>(pdu);
	CVrlinkDisEdoCtrl* pThis = reinterpret_cast<CVrlinkDisEdoCtrl*>(p);

	std::string cName;
	cvTObjAttr attri;
	CPoint3D pt;
	CVector3D t;
	CVector3D l;
	GlobalId id_global;
	pduCrtAdo->getTuple(id_global, cName, attri, pt, t, l);
	pThis->CreateAdoStub(id_global, cName, attri, &pt, &t, &l);
}
void CVrlinkDisEdoCtrl::OnRequest4DeleteAdo( CCustomPdu* pdu, void* p )
{
	CPduDelAdo* pduDelAdo = static_cast<CPduDelAdo*>(pdu);
	CVrlinkDisEdoCtrl* pThis = reinterpret_cast<CVrlinkDisEdoCtrl*>(p);
	pThis->DeleteAdoStub(pduDelAdo->globalId());
}

CVrlinkDisEdoCtrl::CVrlinkDisEdoCtrl(void) : CVrlinkDisDynamic(edo_controller)
{
}


CVrlinkDisEdoCtrl::~CVrlinkDisEdoCtrl(void)
{
}


void CVrlinkDisEdoCtrl::NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self)
{
	CVrlinkDisDynamic::NetworkInitialize(sendTo, receiveFrom, port, self);
	CCustomPdu::StartListening<CPduCrtAdo, (DtPduKind)CCustomPdu::OnCrtAdo>(m_reciver, OnRequest4CreateAdo, this);
	CCustomPdu::StartListening<CPduDelAdo, (DtPduKind)CCustomPdu::OnDelAdo>(m_reciver, OnRequest4DeleteAdo, this);
}

void CVrlinkDisEdoCtrl::NetworkUninitialize()
{
	CVrlinkDisDynamic::NetworkUninitialize();
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnCrtAdo>(m_reciver, OnRequest4CreateAdo, this);
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnDelAdo>(m_reciver, OnRequest4DeleteAdo, this);
}
