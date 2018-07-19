#include "StdAfx.h"
#include "VrlinkDisPedCtrl.h"
#include "cvedstrc.h"
#include <vl/exerciseConnDis.h>
#include <vl/entityPublisherDIS.h>
#include <vl/reflectedEntityList.h>
#include <vl/topoView.h>
#include <vl/reflectedEntityListDIS.h>
#include "vrlinkMath.h"
#include "PduExtObj.h"

CVrlinkDisPedCtrl::CVrlinkDisPedCtrl(void) : CVrlinkDisEdoCtrl(ped_controller)
{

}


CVrlinkDisPedCtrl::~CVrlinkDisPedCtrl(void)
{
}


void CVrlinkDisPedCtrl::Send(IP ip, GlobalId id_global, const cvTObjStateBuf& sb)
{
	EntityPublisher epb;
	bool exists_a_pub = getEntityPub(ip, id_global, epb);
	ASSERT(exists_a_pub);
	if (!exists_a_pub)
		return;
	const cvTObjState* s = (const cvTObjState*)(&sb.state);
	ExternalDriverStateTranLO stateTran;
	TransformLO(s->externalDriverState, stateTran);

	epb.view->setLocation(stateTran.loc);
	epb.view->setOrientation(stateTran.ori);

	CPduExtObj pduObj(id_global, sb.state.externalDriverState);
	epb.cnn->sendStamped(pduObj);

	unsigned char* seg = (unsigned char*)&ip;
	TRACE(TEXT("vrlink: send to [%d.%d.%d.%d]\n"), seg[0], seg[1], seg[2], seg[3]);
}