#pragma once
#include "vrlinkdisdynamic.h"
class CVrlinkDisEdoCtrl :
	public CVrlinkDisDynamic
{
protected:
	CVrlinkDisEdoCtrl(void);
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

private:
	static void OnRequest4CreateAdo( CCustomPdu* pdu, void* pThis );
	static void OnRequest4DeleteAdo( CCustomPdu* pdu, void* pThis );

};

