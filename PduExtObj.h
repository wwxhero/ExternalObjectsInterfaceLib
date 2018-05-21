#pragma once
#include "CustomPdu.h"
#include "objlayout.h"
class CPduExtObj :
	public CCustomPdu
{
	DECLARE_PDU_DYNCREAT(CPduExtObj)
public:
	struct RawState: public CCustomPdu::RawState
	{
		TU16b             visualState;
		TU16b             audioState;
		double            suspStif;     /* suspension stiffness */
		double            suspDamp;     /* suspension dampness */
		double            tireStif;     /* tire stiffness */
		double            tireDamp;     /* tire dampness */
		double            velBrake;
		cvTerQueryHint    posHint[4];
		EDynaFidelity     dynaFidelity; /* vehicle's dynamics fidelity  */
	};
	const RawState& GetState() const
	{
		return m_rs;
	}
public:
	CPduExtObj(GlobalId id_global, const cvTObjState::ExternalDriverState& s) : CCustomPdu((DtPduKind)ExtObjState)
	{
		m_rs.id = id_global;
		m_rs.visualState = s.visualState;
		m_rs.audioState = s.audioState;
		m_rs.suspStif = s.suspStif;
		m_rs.suspDamp = s.suspDamp;
		m_rs.tireStif = s.tireStif;
		m_rs.tireDamp = s.tireDamp;
		m_rs.velBrake = s.velBrake;
		memcpy(m_rs.posHint, s.posHint, 4 * sizeof(cvTerQueryHint));
		m_rs.dynaFidelity = s.dynaFidelity;
		Update(true);
	}
	virtual ~CPduExtObj(void);

#ifdef _DEBUG
	virtual void printData() const;
#endif

private:
	RawState 	m_rs;
};

