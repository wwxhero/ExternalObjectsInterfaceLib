#pragma once

#include <vl/pdu.h>
#include <vl/exerciseConnDIS.h>
#include <vl/entityStatePdu.h>
#include "inetworkdynamic.h"
#include "cveddecl.h"
#include "objlayout.h"
class CPduExtObj;

typedef void (*OnReceiveObj)( CPduExtObj* pdu, void* pThis);
const DtPduKind CPduExtObjKind = DtPduKind(221);

class CPduExtObj :
	public DtPdu
{
public:
	typedef struct ExtObjRawState_tag
	{
		IP                ip;
		TU16b             visualState;
		TU16b             audioState;
		double            suspStif;     /* suspension stiffness */
		double            suspDamp;     /* suspension dampness */
		double            tireStif;     /* tire stiffness */
		double            tireDamp;     /* tire dampness */
		double            velBrake;
		cvTerQueryHint    posHint[4];
		EDynaFidelity     dynaFidelity; /* vehicle's dynamics fidelity  */
	} ExtObjRawState;
	CPduExtObj(IP ip, const cvTObjState::ExternalDriverState& s);
	virtual ~CPduExtObj(void);
	static void StartListening(DtExerciseConn* cnn, OnReceiveObj proc, void* pThis);
	static void StopListening(DtExerciseConn* cnn, OnReceiveObj proc, void* pThis);
	const ExtObjRawState& GetState() const
	{
		return m_rs;
	}
#ifdef _DEBUG
	virtual void printData() const;
#endif
private:
	CPduExtObj(const DtNetPacket *initial, DtBufferPtr buffer, DtPduFactory* pduFactory);
	static DtPdu* create(const DtNetPacket *initial, DtBufferPtr buffer = DtUSE_INTERNAL_BUFFER, DtPduFactory* pduFactory = 0);
	virtual DtPduKind internalGetPduKind() const;
	void Update(bool store);

	ExtObjRawState m_rs;
};

