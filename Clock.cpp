#include "StdAfx.h"
#include "Clock.h"
#include <windows.h>
#include <assert.h>
#define MINUSINFINIT 0x80000001
#define INFINIT 0x7fffffff
//#define _TEST
bool IClock::Less(Tick first, Tick second)
{
#ifdef _TEST
	Tick f = first/5000;
	Tick s = second/5000;
	return f < s;
#else
	return first < second;
#endif
}

Tick IClock::Origin()
{
	return 0;
}

Tick IClock::Infinit()
{
	return INFINIT;
}

Tick IClock::MinusInfinit()
{
	return MINUSINFINIT;
}

Tick IClock::Sub(Tick first, Tick second)
{
	//todo: replace it with asm code
	enum D {minusInfinit, normal, infinit, zero};
	struct stFunc{
		D i1;
		D i2;
		D o;
	} func[] = {
		{minusInfinit, minusInfinit, zero}
		, {minusInfinit, normal, minusInfinit}
		, {minusInfinit, infinit, minusInfinit}
		, {normal, minusInfinit, infinit}
		, {normal, infinit, minusInfinit}
		, {infinit, minusInfinit, infinit}
		, {infinit, normal, infinit}
		, {infinit, infinit, zero}
	};

	stFunc f;
	if (first == INFINIT)
		f.i1 = infinit;
	else if(first == MINUSINFINIT)
		f.i1 = minusInfinit;
	else
		f.i1 = normal;

	if (second == INFINIT)
		f.i2 = infinit;
	else if(second == MINUSINFINIT)
		f.i2 = minusInfinit;
	else
		f.i2 = normal;

	f.o = normal;
	for (int i = 0; i < sizeof(func)/sizeof(stFunc); i ++) {
		if (f.i1 == func[i].i1
			&& f.i2 == func[i].i2) {
			f.o = func[i].o;
			break;
		}
	}

	switch(f.o) {
		case minusInfinit:
			return MINUSINFINIT;
		case normal:
			return first - second;
		case infinit:
			return INFINIT;
		case zero:
			return 0;
		default:
			assert(0);
			return first - second;
	}
}

CClockMock::CClockMock(void)
{
}


CClockMock::~CClockMock(void)
{
}

void CClockMock::StartClock()
{
	m_tick0 = ::GetTickCount();
}
Tick CClockMock::GetTickCnt()
{
	return GetTickCount() - m_tick0;
}


CClockStaticAln::CClockStaticAln(void)
{

}

CClockStaticAln::~CClockStaticAln(void)
{

}

void CClockStaticAln::StartClock()
{
	//the solution bounds the app continous running time to be 24 days
	SYSTEMTIME tm;
	GetSystemTime(&tm);
	struct DigitMS
	{
		Tick v;
		Tick w;
	} msOrg[] = {
		//{tm.wDay, 24}
		  {tm.wHour, 60}
		, {tm.wMinute, 60}
		, {tm.wSecond, 1000}
		, {tm.wMilliseconds, 1}
	};
	Tick& tmOrg = m_tm0;
	tmOrg = 0;
	for (int i = 0; i < sizeof(msOrg)/sizeof(DigitMS); i ++)
	{
		tmOrg += msOrg[i].v;
		tmOrg *= msOrg[i].w;
	}

	m_tick0 = ::GetTickCount();
}


Tick CClockStaticAln::GetTickCnt()
{
	Tick delta = ::GetTickCount() - m_tick0;
	return delta + m_tm0;
}


