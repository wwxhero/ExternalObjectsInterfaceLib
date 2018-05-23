#pragma once
#include "custompdu.h"
class CPduDelAdo :
	public CCustomPdu
{
	DECLARE_PDU_DYNCREAT(CPduDelAdo)
public:
	struct RawState: public CCustomPdu::RawState
	{

	};
public:
	CPduDelAdo(GlobalId id_global) : CCustomPdu((DtPduKind)OnDelAdo)
	{
		m_state.id = id_global;
		Update(true);
	}
	virtual ~CPduDelAdo(void);
	GlobalId globalId()
	{
		return m_state.id;
	}
private:
	CCustomPdu::RawState m_state;
};

