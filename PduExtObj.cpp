#include "stdafx.h"
#include "PduExtObj.h"

IMPLEMENT_PDU_DYNCREATE(CPduExtObj, (DtPduKind)ExtObjState)

BEGIN_UPDATE(CPduExtObj)
	UPDATE_UNIT(m_rs, ip)
	UPDATE_UNIT(m_rs, visualState)
	UPDATE_UNIT(m_rs, audioState)
	UPDATE_UNIT(m_rs, suspStif)
	UPDATE_UNIT(m_rs, suspDamp)
	UPDATE_UNIT(m_rs, tireStif)
	UPDATE_UNIT(m_rs, tireDamp)
	UPDATE_UNIT(m_rs, velBrake)
	UPDATE_UNIT(m_rs, posHint[0].hintState)
	UPDATE_UNIT(m_rs, posHint[0].roadId)
	UPDATE_UNIT(m_rs, posHint[0].roadPiece)
	UPDATE_UNIT(m_rs, posHint[0].intersection)
	UPDATE_UNIT(m_rs, posHint[1].hintState)
	UPDATE_UNIT(m_rs, posHint[1].roadId)
	UPDATE_UNIT(m_rs, posHint[1].roadPiece)
	UPDATE_UNIT(m_rs, posHint[1].intersection)
	UPDATE_UNIT(m_rs, posHint[2].hintState)
	UPDATE_UNIT(m_rs, posHint[2].roadId)
	UPDATE_UNIT(m_rs, posHint[2].roadPiece)
	UPDATE_UNIT(m_rs, posHint[2].intersection)
	UPDATE_UNIT(m_rs, posHint[3].hintState)
	UPDATE_UNIT(m_rs, posHint[3].roadId)
	UPDATE_UNIT(m_rs, posHint[3].roadPiece)
	UPDATE_UNIT(m_rs, posHint[3].intersection)
	UPDATE_UNIT(m_rs, dynaFidelity)
END_UPDATE

CPduExtObj::~CPduExtObj(void)
{
}

#ifdef _DEBUG
void CPduExtObj::printData() const
{
	printf("PostUpdating id:%d"
							", \n\t visualState: [%d] audioState: [%d]"
							", \n\t sus4: [%E, %E, %E, %E]"
							", \n\t velBrake: [%E]"
							", \n\t Fidelity: [%d]"
							"\n"
										, m_rs.ip
										, m_rs.visualState, m_rs.audioState
										, m_rs.suspStif, m_rs.suspDamp, m_rs.tireStif, m_rs.tireDamp
										, m_rs.velBrake
										, m_rs.dynaFidelity);
}
#endif

