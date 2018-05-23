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

	for (std::map<IP, ProxyCnn>::iterator it = m_proxyCnns.begin()
		; it != m_proxyCnns.end()
		; it ++)
	{
		ProxyCnn& proxy = it->second;
		proxy.cnn->sendStamped(crtAdo);

		DtEntityPublisher* pub = new DtEntityPublisher(f18Type, proxy.cnn
				, (DtDeadReckonTypes)s_disConf.drAlgor, DtForceFriendly
				, DtEntityPublisher::guiseSameAsType(), id_vrlink);
		DtEntityStateRepository* esr = pub->entityStateRep();
		DtTopoView* view = new DtTopoView(esr, s_disConf.latitude, s_disConf.longitude);
		view->setOrientation(DtTaitBryan(s_disConf.viewOriPsi, s_disConf.viewOriTheta, s_disConf.viewOriPhi));

		EntityProxy proxyEntity = {pub, view};

		proxy.pubs[id_global.objId] = proxyEntity;
	}
}


void CVrlinkDisAdoCtrl::Notify_OnDelAdo(GlobalId id_global)
{
	CPduDelAdo delAdo(id_global);
	for (std::map<IP, ProxyCnn>::iterator it = m_proxyCnns.begin()
		; it != m_proxyCnns.end()
		; it ++)
	{
		ProxyCnn& proxy = it->second;
		proxy.cnn->sendStamped(delAdo);

		std::map<TObjectPoolIdx, EntityProxy>::iterator itProxyEntity = proxy.pubs.find(id_global.objId);
		if (itProxyEntity != proxy.pubs.end())
		{
			EntityProxy& proxyEntity = itProxyEntity->second;
			delete proxyEntity.view;
			delete proxyEntity.pub;
			proxy.pubs.erase(itProxyEntity);
		}
	}
}