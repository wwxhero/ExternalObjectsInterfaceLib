#include "StdAfx.h"
#include "PduDelAdo.h"
IMPLEMENT_PDU_DYNCREATE(CPduDelAdo, (DtPduKind)OnDelAdo)

BEGIN_UPDATE(CPduDelAdo)
	UPDATE_UNIT(m_state, id.owner)
	UPDATE_UNIT(m_state, id.objId)
END_UPDATE


CPduDelAdo::~CPduDelAdo(void)
{
}
