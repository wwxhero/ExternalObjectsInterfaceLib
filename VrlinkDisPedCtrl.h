#pragma once
#include "vrlinkdisedoctrl.h"
class CVrlinkDisPedCtrl :
	public CVrlinkDisEdoCtrl
{
public:
	CVrlinkDisPedCtrl(void);
	virtual ~CVrlinkDisPedCtrl(void);
	virtual void NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self);
	virtual void NetworkUninitialize();
	virtual void Send(IP ip, GlobalId id_global, const cvTObjState* s);
	virtual void OnNotify_OnTelePdo(GlobalId id_global, CPoint3D* p, CVector3D* t, CVector3D* l) = 0;
private:
	static void OnRequest4TelePdo( CCustomPdu* pdu, void* pThis );
};

