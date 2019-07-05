#include "StdAfx.h"
#include "PduTelePdo.h"
IMPLEMENT_PDU_DYNCREATE(CPduTelePdo, (DtPduKind)OnTelePdo)

BEGIN_UPDATE(CPduTelePdo)
	UPDATE_UNIT(m_tuple, id.owner)
	UPDATE_UNIT(m_tuple, id.objId)
	UPDATE_UNIT(m_tuple, pt.m_x)
	UPDATE_UNIT(m_tuple, pt.m_y)
	UPDATE_UNIT(m_tuple, pt.m_z)
	UPDATE_UNIT(m_tuple, tan.m_i)
	UPDATE_UNIT(m_tuple, tan.m_j)
	UPDATE_UNIT(m_tuple, tan.m_k)
	UPDATE_UNIT(m_tuple, lat.m_i)
	UPDATE_UNIT(m_tuple, lat.m_j)
	UPDATE_UNIT(m_tuple, lat.m_k)
END_UPDATE

CPduTelePdo::~CPduTelePdo(void)
{
}