#include "stdafx.h"
#include "LibExternalObjectIfNetwork.h"
#include "VrlinkDisAdoCtrl.h"
#include "VrlinkDisEdoCtrl.h"
#include "ExternalControlImpl.h"
CVED::IExternalObjectControl* CreateNetworkExternalObjectControl(IMPLE imple, TERMINAL t)
{
	CVED::IExternalObjectControl* p = NULL;
	if (imple == DISVRLINK && t == edo_controller)
		p = new CExternalObjectControlImpl<CVrlinkDisEdoCtrl>();
	else if (imple == DISVRLINK && t == ado_controller)
		p = new CExternalObjectControlImpl<CVrlinkDisAdoCtrl>();
	else
		ASSERT(0); //out of support
	return p;
}

void ReleaseNetworkExternalObjectControl(CVED::IExternalObjectControl* p)
{
	delete p;
}