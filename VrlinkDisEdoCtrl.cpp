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

	EntityStub& buf = pThis->m_reciversStub[id_global];
	buf.updated = false;
	buf.sb = new cvTObjStateBuf;
	memset(buf.sb, 0, sizeof(cvTObjStateBuf));
}
void CVrlinkDisEdoCtrl::OnRequest4DeleteAdo( CCustomPdu* pdu, void* p )
{
	CPduDelAdo* pduDelAdo = static_cast<CPduDelAdo*>(pdu);
	CVrlinkDisEdoCtrl* pThis = reinterpret_cast<CVrlinkDisEdoCtrl*>(p);
	GlobalId id_global = pduDelAdo->globalId();
	pThis->DeleteAdoStub(id_global);

	std::map<GlobalId, EntityStub>::iterator it = pThis->m_reciversStub.find(id_global);
	if (it != pThis->m_reciversStub.end())
	{
		EntityStub buf = it->second;
		delete buf.sb;
		pThis->m_reciversStub.erase(it);
	}
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
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnCrtAdo>(m_reciver, OnRequest4CreateAdo, this);
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnDelAdo>(m_reciver, OnRequest4DeleteAdo, this);
	CVrlinkDisDynamic::NetworkUninitialize();
}