#pragma once
typedef __int32 Tick;

class IClock
{
public:
	virtual void StartClock() = 0;
	virtual Tick GetTickCnt() = 0;
	static bool Less(Tick first, Tick second);
	static Tick Sub(Tick first, Tick second);
	static Tick Origin();
	static Tick Infinit();
	static Tick MinusInfinit();
};

class CClockMock : public IClock
{
public:
	CClockMock(void);
	~CClockMock(void);
	virtual void StartClock();
	virtual Tick GetTickCnt();
private:
	Tick m_tick0;
};

class CClockStaticAln : public IClock
{
public:
	CClockStaticAln();
	virtual ~CClockStaticAln();
	virtual void StartClock();
	virtual Tick GetTickCnt();
private:
	Tick m_tick0;
	Tick m_tm0;
};

