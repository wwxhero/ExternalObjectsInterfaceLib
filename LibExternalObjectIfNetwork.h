#pragma once
#ifndef _LIBEXTERNALOBJECTIFNETWORK_H
#define _LIBEXTERNALOBJECTIFNETWORK_H
#include "ExternalControlInterface.h"

enum IMPLE {IGCOMM = 0, DISVRLINK};
enum TERMINAL {edo_controller = 0, ado_controller};

CVED::IExternalObjectControl* CreateNetworkExternalObjectControl(IMPLE p, TERMINAL t);
void ReleaseNetworkExternalObjectControl(CVED::IExternalObjectControl* p);
#endif