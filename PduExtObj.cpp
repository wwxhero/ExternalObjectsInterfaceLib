#include "stdafx.h"
#include "PduExtObj.h"
#include "utility.h"

CPduExtObj::CPduExtObj(IP ip, const cvTObjState::ExternalDriverState& s)
{
	m_rs.ip = ip;
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

CPduExtObj::CPduExtObj(const DtNetPacket *initial, DtBufferPtr buffer, DtPduFactory* pduFactory)
{
	initPdu(initial, buffer, pduFactory);
	Update(false);
}

CPduExtObj::~CPduExtObj(void)
{
}

DtPduKind CPduExtObj::internalGetPduKind() const
{
	return CPduExtObjKind;
}

DtPdu* CPduExtObj::create(const DtNetPacket *initial, DtBufferPtr buffer, DtPduFactory* pduFactory)
{
   // Return a new'ed instance of this class.
   DtPdu* pdu = new CPduExtObj(initial, buffer, pduFactory);
   return pdu;
}

void CPduExtObj::Update(bool store)
{
	typedef struct _BuffUnit
	{
		unsigned char* addr;
		unsigned int sz;
	} BuffUnit;

#define UNIT(obj, field)\
	{(unsigned char*)(&(obj.field)), sizeof(obj.field)}

#define BUFFUNITS(es)\
	{\
		UNIT(es, ip)\
		, UNIT(es, visualState)\
		, UNIT(es, audioState)\
		, UNIT(es, suspStif)\
		, UNIT(es, suspDamp)\
		, UNIT(es, tireStif)\
		, UNIT(es, tireDamp)\
		, UNIT(es, velBrake)\
		, UNIT(es, posHint[0].hintState)\
		, UNIT(es, posHint[0].roadId)\
		, UNIT(es, posHint[0].roadPiece)\
		, UNIT(es, posHint[0].intersection)\
		, UNIT(es, posHint[1].hintState)\
		, UNIT(es, posHint[1].roadId)\
		, UNIT(es, posHint[1].roadPiece)\
		, UNIT(es, posHint[1].intersection)\
		, UNIT(es, posHint[2].hintState)\
		, UNIT(es, posHint[2].roadId)\
		, UNIT(es, posHint[2].roadPiece)\
		, UNIT(es, posHint[2].intersection)\
		, UNIT(es, posHint[3].hintState)\
		, UNIT(es, posHint[3].roadId)\
		, UNIT(es, posHint[3].roadPiece)\
		, UNIT(es, posHint[3].intersection)\
		, UNIT(es, dynaFidelity)\
	};
#define PDUDTBUFFERPTR (unsigned char*)packet() + sizeof(DtNetPduHeader)

	BuffUnit units[] = BUFFUNITS(m_rs);
	if (store)
	{
		ExtObjRawState mockS;
		unsigned char* packBuffer = (unsigned char*)&mockS;
		unsigned char* w = packBuffer;
		for (int i = 0; i < sizeof(units)/sizeof(BuffUnit); i ++)
		{
			BuffUnit u = units[i];
			memcpy(w, u.addr, u.sz);
			w += u.sz;
		}
		int len = w - packBuffer;
		int minimalSize = sizeof(DtNetPduHeader) + len;
		initPdu(minimalSize, DtUSE_INTERNAL_BUFFER);
		unsigned char* pduBuffer = PDUDTBUFFERPTR;
		memcpy(pduBuffer, packBuffer, len);
	}
	else
	{
		unsigned char* pduBuffer = PDUDTBUFFERPTR;
		for (int i = 0; i < sizeof(units)/sizeof(BuffUnit); i ++)
		{
			BuffUnit u = units[i];
			memcpy(u.addr, pduBuffer, u.sz);
			pduBuffer += u.sz;
		}
	}

#undef PDUDTBUFFERPTR
#undef BUFFUNITS
#undef UNIT

}



void CPduExtObj::StartListening(DtExerciseConn* cnn, OnReceiveObj proc, void* pThis)
{
	cnn->pduFactory()->addCreator(CPduExtObjKind, CPduExtObj::create);
	cnn->addPduCallback(CPduExtObjKind, (DtPduCallbackFcn)proc, pThis);
}

void CPduExtObj::StopListening(DtExerciseConn* cnn, OnReceiveObj proc, void* pThis)
{
	cnn->removePduCallback(CPduExtObjKind, (DtPduCallbackFcn)proc, pThis);
}

#ifdef _DEBUG
void CPduExtObj::printData() const
{
	TRACE(TEXT("Pdu recived from id:%d")
							TEXT(", \n\t visualState: [%d] audioState: [%d]")
							TEXT(", \n\t sus4: [%E, %E, %E, %E]")
							TEXT(", \n\t velBrake: [%E]")
							TEXT(", \n\t Fidelity: [%d]")
							TEXT("\n")
										, m_rs.ip
										, m_rs.visualState, m_rs.audioState
										, m_rs.suspStif, m_rs.suspDamp, m_rs.tireStif, m_rs.tireDamp
										, m_rs.velBrake
										, m_rs.dynaFidelity);
}
#endif
