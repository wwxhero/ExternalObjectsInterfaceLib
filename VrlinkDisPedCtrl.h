#pragma once
#include "vrlinkdisedoctrl.h"
class CVrlinkDisPedCtrl :
	public CVrlinkDisEdoCtrl
{
public:
	CVrlinkDisPedCtrl(void);
	virtual ~CVrlinkDisPedCtrl(void);
	virtual void Send(IP ip, GlobalId id_global, const cvTObjState* s);
};

