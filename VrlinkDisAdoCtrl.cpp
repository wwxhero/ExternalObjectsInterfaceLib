#include "StdAfx.h"

#include <vl/exerciseConnDis.h>
#include <vl/entityPublisherDIS.h>
#include <vl/reflectedEntityList.h>
#include <vl/topoView.h>
#include <vl/reflectedEntityListDIS.h>

#include <vl/exerciseConnInitializer.h>
#include <vlpi/entityTypes.h>

#include <vl/entityStateRepository.h>
#include <vl/reflectedEntity.h>

#include "VrlinkDisAdoCtrl.h"
#include "PduCrtAdo.h"
#include "PduDelAdo.h"
#include "PduTelePdo.h"
#include "utility.h"


CVrlinkDisAdoCtrl::CVrlinkDisAdoCtrl(void) : CVrlinkDisDynamic(ado_controller)
{
}


CVrlinkDisAdoCtrl::~CVrlinkDisAdoCtrl(void)
{
}


void CVrlinkDisAdoCtrl::Notify_OnNewAdo(GlobalId id_global
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l)
{
	CPduCrtAdo crtAdo(id_global, szName, cAttr, pos, t, l);

	DtEntityType f18Type(s_disConf.kind, s_disConf.domain,
		s_disConf.country, s_disConf.category, s_disConf.subCategory, s_disConf.specific, s_disConf.extra);
	DtObjectId id_vrlink = GlobalId2VrlinkId(id_global);

	for (std::map<IP, CnnOut>::iterator it = m_cnnsOut.begin()
		; it != m_cnnsOut.end()
		; it ++)
	{
		CnnOut& out = it->second;
		out.cnn->sendStamped(crtAdo);

		DtEntityPublisher* pub = new DtEntityPublisher(f18Type, out.cnn
				, (DtDeadReckonTypes)s_disConf.drAlgor, DtForceFriendly
				, DtEntityPublisher::guiseSameAsType(), id_vrlink);
		DtEntityStateRepository* esr = pub->entityStateRep();
		DtTopoView* view = new DtTopoView(esr, s_disConf.latitude, s_disConf.longitude);
		view->setOrientation(DtTaitBryan(s_disConf.viewOriPsi, s_disConf.viewOriTheta, s_disConf.viewOriPhi));

		EntityPub epb = {pub, view};

		out.pubs[id_global.objId] = epb;
	}
}


void CVrlinkDisAdoCtrl::Notify_OnDelAdo(GlobalId id_global)
{
	CPduDelAdo delAdo(id_global);
	for (std::map<IP, CnnOut>::iterator itCnn = m_cnnsOut.begin()
		; itCnn != m_cnnsOut.end()
		; itCnn ++)
	{
		CnnOut& out = itCnn->second;
		out.cnn->sendStamped(delAdo);

		std::map<TObjectPoolIdx, EntityPub>::iterator itPub = out.pubs.find(id_global.objId);
		if (itPub != out.pubs.end())
		{
			EntityPub& epb = itPub->second;
			delete epb.view;
			delete epb.pub;
			out.pubs.erase(itPub);
		}
	}
}

void CVrlinkDisAdoCtrl::Notify_OnTelePDO(GlobalId id_global, const CPoint3D& pos, const CVector3D& tan, const CVector3D& lat)
{
	CPduTelePdo telePdo(id_global, pos, tan, lat);
	for (std::map<IP, CnnOut>::iterator itCnn = m_cnnsOut.begin()
		; itCnn != m_cnnsOut.end()
		; itCnn ++)
	{
		CnnOut& out = itCnn->second;
		out.cnn->sendStamped(telePdo);
	}
	unsigned char* seg = (unsigned char*)&id_global.owner;
	TRACE(TEXT("\nCVrlinkDisAdoCtrl::Notify_OnTelePDO:[%d.%d.%d.%d]  %d")
			TEXT("\n\tp:%10.2f %10.2f %10.2f")
			TEXT("\n\tt:%10.2f %10.2f %10.2f")
			TEXT("\n\tl:%10.2f %10.2f %10.2f")
		, seg[0], seg[1], seg[2], seg[3], id_global.objId
		, pos.m_x, pos.m_y, pos.m_z
		, tan.m_i, tan.m_j, tan.m_k
		, lat.m_i, lat.m_j, lat.m_k);
}