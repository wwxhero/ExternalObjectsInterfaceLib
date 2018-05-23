#include "StdAfx.h"
#include <assert.h>
#include "cvedstrc.h"
#include "RealtimeNetworkDynamic.h"
#include "hcsmobject.h"
#include "utility.h"


inline void InitBuffer(NObjSb& buff)
{
	buff.stamp = IClock::Origin();
	buff.sb = (const cvTObjStateBuf*)malloc(sizeof(cvTObjStateBuf));
}

inline void UninitBuffer(NObjSb& buff)
{
	free((void *)buff.sb);
}

inline SQNObjSb* GenSQNObjSb(QNObjSb* q)
{
	SQNObjSb* sq = (SQNObjSb*)malloc(sizeof(SQNObjSb));
	sq->queSb = q;
	sq->lock = 1;
	return sq;
}

inline void ReleaseSQNObjSb(SQNObjSb* sq)
{
	free(sq);
}

class CLocker
{
public:
	inline CLocker(SQNObjSb& sq, bool autoUnlock = true)
		: m_lockee(sq)
		, m_autoUnlock(autoUnlock)
	{

	}
	inline ~CLocker()
	{
		if (m_autoUnlock)
			V(m_lockee);
	}
	inline QNObjSb* Lock()
	{
		return P(m_lockee);
	}
	inline void UnLock()
	{
		V(m_lockee);
	}
private:
	//P operation
	inline QNObjSb* P(SQNObjSb& sq)
	{
		while (--sq.lock < 0)
		{
			sq.lock ++;
			::Sleep(0); // yeild the cpu
		}
		return sq.queSb;
	}
	//V operation
	inline void V(SQNObjSb& sq)
	{
		sq.lock ++;
	}

	SQNObjSb& m_lockee;
	bool m_autoUnlock;
};

void CRealtimeNetworkDynamic::OnCmdUpdateStateBuffer(int id, int size, IGComm::Tbyte* data, void* param)
{
	if (size > 0)
	{
		assert(id == UpdateStateBuffer);
		CRealtimeNetworkDynamic* pThis = reinterpret_cast<CRealtimeNetworkDynamic*>(param);
		IGComm::CRealtimeDataReceiver* pReceiver = &pThis->m_rcv;
		NObjSb sb;
		InitBuffer(sb);
		sb.stamp = pReceiver->GetCurrentFbsFrameNumber();
		pThis->DeserializeExternalObj(data, size, const_cast<cvTObjStateBuf *>(sb.sb));

		SOCKADDR_IN rcvAddr;
		pReceiver->GetSenderAddr(rcvAddr);
		IP ip = rcvAddr.sin_addr.S_un.S_addr;
#ifdef _DEBUG
		unsigned char* seg = (unsigned char*)&ip;
		TRACE(TEXT("receive from: %d.%d.%d.%d\n"), seg[0], seg[1], seg[2], seg[3]);
#endif

		//SQNObjSb* sq = pThis->m_mapIp2Qb[ip];
		MapIp2Qb::iterator it = pThis->m_mapIp2Qb.find(ip);
		if (it != pThis->m_mapIp2Qb.end())
		{
			SQNObjSb* sq = it->second;
			CLocker l(*sq);
			QNObjSb* q = l.Lock();
			q->push_back(sb);
		}

	}
}

CRealtimeNetworkDynamic::CRealtimeNetworkDynamic(void)
{
}


CRealtimeNetworkDynamic::~CRealtimeNetworkDynamic(void)
{
}

void CRealtimeNetworkDynamic::NetworkInitialize(const std::list<IP>& senders, const std::list<IP>& recievers, int port, IP self)
{
	WSADATA        wsd;
   	bool init = (WSAStartup(MAKEWORD(2, 2), &wsd) == 0);
   	assert(init);
   	m_clk.StartClock();

	//the process for initilizing receivers
	init = m_rcv.Init(port);
	assert(init);
	m_rcv.RegisterIgCommandCallBack(UpdateStateBuffer, OnCmdUpdateStateBuffer, this);
	for (std::list<IP>::const_iterator it = recievers.begin(); it != recievers.end(); it ++)
	{
		IP ip = *it;
		QNObjSb* q = new QNObjSb();
		SQNObjSb* sq = GenSQNObjSb(q);
		m_mapIp2Qb.insert(std::pair<IP, SQNObjSb*>(ip, sq));
	}

	//the process for initializing senders
	for (std::list<IP>::const_iterator it = senders.begin(); it != senders.end(); it ++)
	{
		IP ip = *it;
		IGComm::CRealtimeDataSender* sender = new IGComm::CRealtimeDataSender();
		init = sender->Init(port, ip);
		assert(init);
		m_sndrs.insert(std::pair<IP, IGComm::CRealtimeDataSender*>(ip, sender));
		char* buffer = (char*)malloc(sizeof(cvTObjStateBuf));
		m_sndbuffer.insert(std::pair<IP,char*>(ip, buffer));
	}


}
void CRealtimeNetworkDynamic::NetworkUninitialize()
{
	m_rcv.Uninit();
	for (MapIp2Qb::iterator it = m_mapIp2Qb.begin(); it != m_mapIp2Qb.end(); )
	{
		SQNObjSb* sq = it->second;
		CLocker l(*sq, false);
		QNObjSb* q = l.Lock();
		while(!q->empty())
		{
			NObjSb sbn = q->front();
			UninitBuffer(sbn);
			q->pop_front();
		}
		ReleaseSQNObjSb(sq);
		delete q;
		it = m_mapIp2Qb.erase(it);
	}
	m_mapIp2Qb.clear();

	for (std::map<IP, IGComm::CRealtimeDataSender *>::iterator it = m_sndrs.begin(); it != m_sndrs.end(); it ++)
	{
		IGComm::CRealtimeDataSender* sender = it->second;
		delete sender;
	}
	m_sndrs.clear();

	for (std::map<IP, char*>::iterator it = m_sndbuffer.begin(); it != m_sndbuffer.end(); it ++)
	{
		char* buffer = it->second;
		free(buffer);
	}
	m_sndbuffer.clear();
}
void CRealtimeNetworkDynamic::Send(IP ip, const cvTObjStateBuf& sb)
{
	IGComm::CRealtimeDataSender* sender = m_sndrs[ip];
	int tCap = sizeof(cvTObjStateBuf);
	char* tBuffer = m_sndbuffer[ip];
	int tLen = 0;
	char* uBuffer = tBuffer;

	int uLen = SerializeExternalObj(&sb, uBuffer);
	uBuffer += uLen;

	tLen = uBuffer - tBuffer;
	assert(tLen <= tCap);

	sender->SetFrameNumber(m_clk.GetTickCnt());
	sender->AddCommand(UpdateStateBuffer, tLen, (IGComm::Tbyte*)tBuffer);
	sender->Broadcast();
}
bool CRealtimeNetworkDynamic::Receive(IP ip, const cvTObjStateBuf*& sb)
{
	SQNObjSb* sq = m_mapIp2Qb[ip];
	assert(NULL != sq);
	CLocker l(*sq);
	QNObjSb* queue = l.Lock(); //it can be modified to Lock(milliseconds) to improve the better visual experience
	if (NULL == queue
		|| queue->empty())
		return false;
	else
	{
		assert(NULL != queue);
		TRACE(TEXT("Pre-receive:queue size [%d]\n"), queue->size());
		Tick cT = m_clk.GetTickCnt();
		NObjSb sbn = queue->front();
		NObjSb sbnm1 = {IClock::MinusInfinit(), NULL};
		while (IClock::Less(sbn.stamp, cT))
		{
			queue->pop_front();
			UninitBuffer(sbnm1);
			sbnm1 = sbn;
			if (queue->empty())
			{
				sbn.stamp = IClock::Infinit();
				sbn.sb = NULL;
				break;
			}
			else
				sbn = queue->front();
		}

		Tick d = IClock::Sub(cT, sbnm1.stamp);
		Tick dPrime = IClock::Sub(sbn.stamp, cT);

		if (IClock::Less(d, dPrime))
		{
			queue->push_front(sbnm1);
			sb = sbnm1.sb;
		}
		else
		{
			UninitBuffer(sbnm1);
			sb = sbn.sb;
		}
		assert(NULL != sb);
		TRACE(TEXT("Post-receive:queue size [%d]\n"), queue->size());
		return true;
	}
}

typedef struct _BuffUnit
{
	unsigned char* addr;
	unsigned int sz;
} BuffUnit;

#define UNIT(obj, field)\
	{(unsigned char*)(&(obj.field)), sizeof(obj.field)}

#define BUFFUNITS(es)\
	{\
		  UNIT(es, position.x)\
		, UNIT(es, position.y)\
		, UNIT(es, position.z)\
		, UNIT(es, tangent.i)\
		, UNIT(es, tangent.j)\
		, UNIT(es, tangent.k)\
		, UNIT(es, lateral.i)\
		, UNIT(es, lateral.j)\
		, UNIT(es, lateral.k)\
		, UNIT(es, vel)\
		, UNIT(es, visualState)\
		, UNIT(es, audioState)\
		, UNIT(es, acc)\
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
		, UNIT(es, latAccel)\
		, UNIT(es, dynaFidelity)\
		, UNIT(es, angularVel.i)\
		, UNIT(es, angularVel.j)\
		, UNIT(es, angularVel.k)\
	};

int CRealtimeNetworkDynamic::SerializeExternalObj(const cvTObjStateBuf* stateBuff, char* buffer) const
{
	const cvTObjState& s = stateBuff->state;
	const cvTObjState::ExternalDriverState& es = s.externalDriverState;
	//const contInp& inp = stateBuff->contInp;

	BuffUnit units[] = BUFFUNITS(es);

	unsigned char* w = (unsigned char*)buffer;
	for (int i = 0; i < sizeof(units)/sizeof(BuffUnit); i ++)
	{
		BuffUnit u = units[i];
		memcpy(w, u.addr, u.sz);
		w += u.sz;
	}

	int l =  w - (unsigned char*)buffer;

	//DumpBuffer(TEXT("Serialize"), (unsigned char*)buffer, l);

	return l;
}

void CRealtimeNetworkDynamic::DeserializeExternalObj(const char* buffer, int buflen, cvTObjStateBuf* stateBuff) const
{

	const cvTObjState& s = stateBuff->state;
	const cvTObjState::ExternalDriverState& es = s.externalDriverState;
	//const contInp& inp = stateBuff->contInp;

	BuffUnit units[] = BUFFUNITS(es);

	unsigned char* r = (unsigned char*)buffer;
	for (int i = 0; i < sizeof(units)/sizeof(BuffUnit); i ++)
	{
		BuffUnit u = units[i];
		memcpy(u.addr, r, u.sz);
		r += u.sz;
	}

	assert(buflen == r - (unsigned char*)buffer);
}

#undef UNIT
#undef BUFFUNITS

