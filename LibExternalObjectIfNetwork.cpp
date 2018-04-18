#include "stdafx.h"
#include "LibExternalObjectIfNetwork.h"
#include "ExternalControlImpl.h"
CVED::IExternalObjectControl* CreateNetworkExternalObjectControl(BASETYPE t)
{
	CVED::IExternalObjectControl* p = NULL;
	switch(t)
	{
		case IGCOMM:
			p = new CExternalObjectControlImpl<CRealtimeNetworkDynamic>();
			break;
		case DISVRLINK:
			p = new CExternalObjectControlImpl<CVrlinkDisDynamic>();
			break;
	}
	return p;
}

void ReleaseNetworkExternalObjectControl(CVED::IExternalObjectControl* p)
{
	delete p;
}