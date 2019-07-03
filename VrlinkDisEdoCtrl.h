#pragma once
#include "vrlinkdisdynamic.h"
#include "utility.h"
class CVrlinkDisEdoCtrl :
	public CVrlinkDisDynamic
{
protected:
	CVrlinkDisEdoCtrl(void);
	CVrlinkDisEdoCtrl(TERMINAL t);
	virtual ~CVrlinkDisEdoCtrl(void);
	virtual void NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self);
	virtual void NetworkUninitialize();

	virtual void CreateAdoStub(GlobalId id_global
							, const std::string& name
							, const cvTObjAttr& cAttr
							, const CPoint3D* cpInitPos
							, const CVector3D* cpInitTran
							, const CVector3D* cpInitLat) = 0;
	virtual void DeleteAdoStub(GlobalId id_global) = 0;

	virtual void Notify_OnNewAdo(GlobalId id
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l)
	{
		//EDO controllor does not notify for creating an ado
		ASSERT(0);
	}
	virtual void Notify_OnDelAdo(GlobalId id)
	{
		//EDO controller does not notify for deleting an ado
		ASSERT(0);
	}
	virtual void Notify_OnTelePDO(GlobalId id_local, const CPoint3D& pos, const CVector3D& tan, const CVector3D& lat)
	{
		//EDO controller does not notify for teleporting a PDO
		ASSERT(0);
	}
private:
	static void OnRequest4CreateAdo( CCustomPdu* pdu, void* pThis );
	static void OnRequest4DeleteAdo( CCustomPdu* pdu, void* pThis );

};

