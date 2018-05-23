#pragma once
#include "vrlinkdisdynamic.h"
class CVrlinkDisAdoCtrl :
	public CVrlinkDisDynamic
{
public:
	CVrlinkDisAdoCtrl(void);
	virtual ~CVrlinkDisAdoCtrl(void);

protected:
	virtual void Notify_OnNewAdo(GlobalId id_local
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l);


	virtual void Notify_OnDelAdo(GlobalId id_local);

};

