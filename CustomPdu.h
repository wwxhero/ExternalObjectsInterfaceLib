#pragma once
#include <vl/pdu.h>
#include <vl/exerciseConnDIS.h>
#include <vl/entityStatePdu.h>
#include "INetworkDynamic.h"
#include "hcsmobject.h"
#include "cvedstrc.h"

class CCustomPdu;
typedef void (*OnReceivePdu)( CCustomPdu* pdu, void* pThis);


class CCustomPdu : public DtPdu
{
public:
	enum
	{
		ExtObjState = DtPduKind(221)
		, OnCreateObj = DtPduKind(222)
	};
	typedef struct _BuffUnit
	{
		unsigned char* addr;
		unsigned int sz;
	} BuffUnit;
	struct RawState
	{
		GlobalId id;
	};
	CCustomPdu(DtPduKind t) : m_type(t)
	{
	}
private:
	virtual DtPduKind internalGetPduKind() const
	{
		return m_type;
	}
	DtPduKind Kind() const
	{
		return m_type;
	}
protected:
	template<typename TRawState>
	void Update(bool store, BuffUnit* units, unsigned int num)
	{
#define PDUDTBUFFERPTR (unsigned char*)packet() + sizeof(DtNetPduHeader)
		if (store)
		{
			TRawState mockS;
			unsigned char* packBuffer = (unsigned char*)&mockS;
			unsigned char* w = packBuffer;
			for (int i = 0; i < num; i ++)
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
			for (int i = 0; i < num; i ++)
			{
				BuffUnit u = units[i];
				memcpy(u.addr, pduBuffer, u.sz);
				pduBuffer += u.sz;
			}
		}
#undef PDUDTBUFFERPTR
	}
public:
	template<typename PduCls, DtPduKind kind>
	static void StartListening(DtExerciseConn* cnn, OnReceivePdu proc, void* pThis)
	{
		cnn->pduFactory()->addCreator(kind, PduCls::create);
		cnn->addPduCallback(kind, (DtPduCallbackFcn)proc, pThis);
	}

	template<DtPduKind kind>
	static void StopListening(DtExerciseConn* cnn, OnReceivePdu proc, void* pThis)
	{
		cnn->removePduCallback(kind, (DtPduCallbackFcn)proc, pThis);
	}

private:
	DtPduKind m_type;
};

#define DECLARE_PDU_DYNCREAT(CustomDesc)\
	public:\
	friend class CCustomPdu;\
	static DtPdu* create(const DtNetPacket *initial, DtBufferPtr buffer = DtUSE_INTERNAL_BUFFER, DtPduFactory* pduFactory = 0);\
	private:\
	CustomDesc();\
	void Update(bool store);

#define IMPLEMENT_PDU_DYNCREATE(CustomDesc, kind)\
	DtPdu* CustomDesc::create(const DtNetPacket *initial, DtBufferPtr buffer, DtPduFactory* pduFactory)\
	{\
		CustomDesc* pdu = new CustomDesc();\
   		pdu->initPdu(initial, buffer, pduFactory);\
   		pdu->Update(false);\
   		return pdu;\
	}\
	CustomDesc::CustomDesc() : CCustomPdu(kind)\
	{\
	}

#define UPDATE_UNIT(obj, field)\
	{(unsigned char*)(&(obj.field)), sizeof(obj.field)},

#define BEGIN_UPDATE(CustomDesc)\
	void CustomDesc::Update(bool store)\
	{\
		BuffUnit units[] = {

#define END_UPDATE\
		};\
		CCustomPdu::Update<RawState>(store, units, sizeof(units)/sizeof(BuffUnit));\
	}

