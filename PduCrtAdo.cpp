#include "StdAfx.h"
#include "PduCrtAdo.h"
IMPLEMENT_PDU_DYNCREATE(CPduCrtAdo, (DtPduKind)OnCrtAdo)

BEGIN_UPDATE(CPduCrtAdo)
	UPDATE_UNIT(m_tuple, id.owner)
	UPDATE_UNIT(m_tuple, id.objId)
	UPDATE_UNIT(m_tuple, name)
	UPDATE_UNIT(m_tuple, attri.solId)
	UPDATE_UNIT(m_tuple, attri.hcsmId)
	UPDATE_UNIT(m_tuple, attri.xSize)
	UPDATE_UNIT(m_tuple, attri.ySize)
	UPDATE_UNIT(m_tuple, attri.zSize)
	UPDATE_UNIT(m_tuple, attri.cigi)
	UPDATE_UNIT(m_tuple, attri.colorIndex)
	UPDATE_UNIT(m_tuple, attri.frictionCoeff)
	UPDATE_UNIT(m_tuple, attri.bounceEnergyLoss)
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

CPduCrtAdo::~CPduCrtAdo(void)
{
}
