#include "StdAfx.h"
#include <vl/exerciseConnDis.h>
#include <vl/entityPublisherDIS.h>
#include <vl/topoView.h>
#include "VrlinkDisEdoCtrl.h"
#include "PduCrtAdo.h"
#include "PduDelAdo.h"
#include "utility.h"



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

	EntityState& buf = pThis->m_statesIn[id_global];
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

	std::map<GlobalId, EntityState>::iterator it = pThis->m_statesIn.find(id_global);
	ASSERT(it != pThis->m_statesIn.end());
	if (it != pThis->m_statesIn.end())
	{
		EntityState& buf = it->second;
		delete buf.sb;
		pThis->m_statesIn.erase(it);
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
	GlobalId id_global_self = {self, 0};
	DtEntityType f18Type(s_disConf.kind, s_disConf.domain,
		s_disConf.country, s_disConf.category, s_disConf.subCategory, s_disConf.specific, s_disConf.extra);
	for (std::list<IP>::const_iterator itOut = sendTo.begin()
		; itOut != sendTo.end()
		; itOut ++)
	{
		ASSERT(m_cnnsOut.find(*itOut) != m_cnnsOut.end());
		CnnOut& out = m_cnnsOut[*itOut];
		DtEntityPublisher* pub = new DtEntityPublisher(f18Type, out.cnn
			, (DtDeadReckonTypes)s_disConf.drAlgor, DtForceFriendly
			, DtEntityPublisher::guiseSameAsType(), GlobalId2VrlinkId(id_global_self));
		DtEntityStateRepository* esr = pub->entityStateRep();
		DtTopoView* view = new DtTopoView(esr, s_disConf.latitude, s_disConf.longitude);
		view->setOrientation(DtTaitBryan(s_disConf.viewOriPsi, s_disConf.viewOriTheta, s_disConf.viewOriPhi));
		EntityPub epb = {pub, view};
		out.pubs[0] = epb;
	}

	CCustomPdu::StartListening<CPduCrtAdo, (DtPduKind)CCustomPdu::OnCrtAdo>(m_cnnIn, OnRequest4CreateAdo, this);
	CCustomPdu::StartListening<CPduDelAdo, (DtPduKind)CCustomPdu::OnDelAdo>(m_cnnIn, OnRequest4DeleteAdo, this);
}

void CVrlinkDisEdoCtrl::NetworkUninitialize()
{
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnCrtAdo>(m_cnnIn, OnRequest4CreateAdo, this);
	CCustomPdu::StopListening<(DtPduKind)CCustomPdu::OnDelAdo>(m_cnnIn, OnRequest4DeleteAdo, this);
	CVrlinkDisDynamic::NetworkUninitialize();
}
