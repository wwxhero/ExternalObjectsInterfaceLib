#pragma once
#include "vrlinkdisedoctrl.h"
class CVrlinkDisPedCtrl :
	public CVrlinkDisEdoCtrl
{
public:
	CVrlinkDisPedCtrl(void);
	virtual ~CVrlinkDisPedCtrl(void);
	virtual void NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self);
};

