#pragma once
#include "custompdu.h"
class CPduCrtAdo :
	public CCustomPdu
{
	DECLARE_PDU_DYNCREAT(CPduCrtAdo)
public:
	struct RawState: public CCustomPdu::RawState
	{
		char name[PDU_STRLEN];
		cvTObjAttr attri;
		CPoint3D pt;
		CVector3D tan;
		CVector3D lat;
	};
public:
	CPduCrtAdo(GlobalId id_global, const char* szName, const cvTObjAttr& attri, const CPoint3D& pt, const CVector3D& tan, const CVector3D& lat)
		: CCustomPdu((DtPduKind)OnCrtAdo)
	{
		m_tuple.id = id_global;
		char * p_d = m_tuple.name;
		for (const char* p_s = szName
			; *p_s != '\0' && p_d - m_tuple.name < PDU_STRLEN-1
			; p_s ++, p_d ++)
			*p_d = *p_s;
		*p_d = '\0'; //trancade will happen
		m_tuple.attri = attri;
		m_tuple.pt = pt;
		m_tuple.tan = tan;
		m_tuple.lat = lat;
		Update(true);
	}
	virtual ~CPduCrtAdo(void);
	void getTuple(GlobalId& id_global, std::string& name, cvTObjAttr& attri, CPoint3D& pt, CVector3D& tan, CVector3D& lat)
	{
		id_global = m_tuple.id;
		name = m_tuple.name;
		attri = m_tuple.attri;
		pt = m_tuple.pt;
		tan = m_tuple.tan;
		lat = m_tuple.lat;
	}
private:
	RawState m_tuple;
};


