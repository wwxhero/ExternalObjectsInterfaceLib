#pragma once
#include "INetworkDynamic.h"
#include "igcomm.h"
#include <queue>
#include <map>
#include <boost\crc.hpp>
#include "Clock.h"
#include "LeftistHeap.h"


struct cvTObjStateBuf;
struct NObjSb
{
	Tick stamp;
	const cvTObjStateBuf* sb;
};

struct NObjSbComp
{
	static int Compare(const NObjSb& first, const NObjSb& second)
	{
		if (IClock::Less(first.stamp, second.stamp))
			return -1;
		else if(IClock::Less(second.stamp, first.stamp))
			return +1;
		else
			return 0;
	}
};

//typedef std::deque<NObjSb> QNObjSb;
typedef LeftistHeap<NObjSb, NObjSbComp> QNObjSb;

struct SQNObjSb
{
	boost::atomic<int> lock;
	QNObjSb* queSb;
};

typedef std::map<IP, SQNObjSb*> MapIp2Qb;

//typedef void(*CommandCallBack) (int,int,Tbyte*, void* param);


class CRealtimeNetworkDynamic :
	public INetworkDynamic
{
	enum Cmd{
		UpdateStateBuffer = 0
	};
public:
	CRealtimeNetworkDynamic(void);
	virtual ~CRealtimeNetworkDynamic(void);
	virtual void NetworkInitialize(const std::list<IP>& senders, const std::list<IP>& receivers, int port, IP self);
	virtual void NetworkUninitialize();
	virtual void PreDynaCalc(){};
	virtual void Send(IP ip, const cvTObjStateBuf& sb);  //if ip is a cluster mask, it broadcasts to the subnet
	virtual bool Receive(IP ip, const cvTObjStateBuf*& sb);
	virtual void PostDynaCalc(){};
private:
	int SerializeExternalObj(const cvTObjStateBuf* stateBuff, char* buffer) const;
	void DeserializeExternalObj(const char* buffer, int buflen, cvTObjStateBuf* stateBuff) const;

	static void OnCmdUpdateStateBuffer(int, int, IGComm::Tbyte*, void*);
	IGComm::CRealtimeDataReceiver m_rcv;
	std::map<IP, IGComm::CRealtimeDataSender *> m_sndrs;
	std::map<IP, char*> m_sndbuffer;
	MapIp2Qb m_mapIp2Qb;

	CClockStaticAln m_clk;
};

