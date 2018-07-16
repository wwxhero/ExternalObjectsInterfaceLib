#include "StdAfx.h"
#include "VrlinkDisPedCtrl.h"


CVrlinkDisPedCtrl::CVrlinkDisPedCtrl(void) : CVrlinkDisEdoCtrl(ped_controller)
{

}


CVrlinkDisPedCtrl::~CVrlinkDisPedCtrl(void)
{
}


void CVrlinkDisPedCtrl::NetworkInitialize(const std::list<IP>& sendTo, const std::list<IP>& receiveFrom, int port, IP self)
{
	//CVrlinkDisDynamic::NetworkInitialize(sendTo, receiveFrom, port, self);
	CVrlinkDisEdoCtrl::NetworkInitialize(sendTo, receiveFrom, port, self);
}