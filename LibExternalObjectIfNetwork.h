#pragma once
#ifndef _LIBEXTERNALOBJECTIFNETWORK_H
#define _LIBEXTERNALOBJECTIFNETWORK_H
#include "ExternalControlInterface.h"

enum BASETYPE {IGCOMM, DISVRLINK};

CVED::IExternalObjectControl* CreateNetworkExternalObjectControl(BASETYPE t);
void ReleaseNetworkExternalObjectControl(CVED::IExternalObjectControl* p);
#endif