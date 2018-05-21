#include "StdAfx.h"
#include "VrlinkDisEdoCtrl.h"

void CVrlinkDisEdoCtrl::OnRequest4CreateAdo( CCustomPdu* pdu, void* p )
{
	CPduCrtAdo* pduCrtAdo = static_cast<CCustomPdu*>(pdu);
	CVrlinkDisEdoCtrl* pThis = reinterpret_cast<CVrlinkDisEdoCtrl*>(p);

	std::string cName;
	cvTObjAttr attri;
	CPoint3D p;
	CVector3D t;
	CVector3D l;
	GlobalId id_global;
	pduCrtAdo->getTuple(id_global, cName, attri, p, t, l);
	pThis->CreateAdoStub(id_global, cName, attri, p, t, l);
}
void CVrlinkDisEdoCtrl::OnRequest4DeleteAdo( CCustomPdu* pdu, void* p )
{
	CPduDelAdo* pduDelAdo = static_cast<CCustomPdu*>(pdu);
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
	CCustomPdu::StartListening<CPduCrtAdo, (DtPduKind)CCustomPdu::PduCrtAdo>(m_reciver, OnRequest4CreateAdo, this);
	CCustomPdu::StartListening<CPduDelAdo, (DtPduKind)CCustomPdu::PduDelAdo>(m_reciver, OnRequest4DeleteAdo, this);
}

void CVrlinkDisEdoCtrl::NetworkUninitialize()
{
	CVrlinkDisDynamic::NetworkUninitialize(sendTo, receiveFrom, port, self);
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::PduCrtAdo>(m_reciver, OnRequest4CreateAdo, this);
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::PduDelAdo>(m_reciver, OnRequest4DeleteAdo, this);
}
