#pragma once
#include "vrlinkdisdynamic.h"
class CVrlinkDisAdoCtrl :
	public CVrlinkDisDynamic
{
public:
	CVrlinkDisAdoCtrl(void);
	virtual ~CVrlinkDisAdoCtrl(void);

protected:
	virtual void Notify_OnNewAdo(GlobalId id_global
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l);


	virtual void Notify_OnDelAdo(GlobalId id_global);
	virtual void Notify_OnTelePDO(GlobalId id_global, const CPoint3D& pos, const CVector3D& tan, const CVector3D& lat);
};

