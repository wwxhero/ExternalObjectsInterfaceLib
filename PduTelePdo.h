#pragma once
#include "custompdu.h"
class CPduTelePdo :
	public CCustomPdu
{
	DECLARE_PDU_DYNCREAT(CPduTelePdo)
public:
	struct RawState: public CCustomPdu::RawState
	{
		CPoint3D pt;
		CVector3D tan;
		CVector3D lat;
	};
public:
	CPduTelePdo(GlobalId id_global, const CPoint3D& pt, const CVector3D& tan, const CVector3D& lat)
		: CCustomPdu((DtPduKind)OnTelePdo)
	{
		m_tuple.id = id_global;
		m_tuple.pt = pt;
		m_tuple.tan = tan;
		m_tuple.lat = lat;
		Update(true);
	}
	virtual ~CPduTelePdo(void);
	void getTuple(GlobalId& id_global, CPoint3D& pt, CVector3D& tan, CVector3D& lat)
	{
		id_global = m_tuple.id;
		pt = m_tuple.pt;
		tan = m_tuple.tan;
		lat = m_tuple.lat;
	}
private:
	RawState m_tuple;
};


