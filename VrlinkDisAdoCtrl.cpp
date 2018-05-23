#include "StdAfx.h"
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
	for (std::map<IP, ProxyCnn>::iterator it = m_proxyCnns.begin()
		; it != m_proxyCnns.end()
		; it ++)
	{
		ProxyCnn& proxy = it->second;
		proxy.cnn->sendStamped(crtAdo);
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
	}
}