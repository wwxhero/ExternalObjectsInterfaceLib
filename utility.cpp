#include "stdafx.h"
#include "utility.h"
#include <Windows.h>
#include <malloc.h>
void __cdecl Logoutf(const TCHAR* format, ...)
{
#ifdef _DEBUG
	va_list arglist;
	va_start(arglist, format);
	int cap = 256;
	TCHAR* buffer = (TCHAR *)malloc(sizeof(TCHAR) * cap);
	bool gen = false;
	do
	{
		int l = _vsntprintf(buffer, cap, format, arglist);
		gen = (l > 0 && l < cap);
		if (!gen)
		{
			cap += cap;
			buffer = (TCHAR *)realloc(buffer, sizeof(TCHAR) * cap);
		}
	} while(!gen);

	OutputDebugString(buffer);
	free(buffer);
#endif
}

void DumpBuffer(const TCHAR* name, const unsigned char* addr, unsigned int l)
{
	const TCHAR code[16] = {TEXT('0'), TEXT('1'), TEXT('2'), TEXT('3')
						  , TEXT('4'), TEXT('5'), TEXT('6'), TEXT('7')
						  , TEXT('8'), TEXT('9'), TEXT('A'), TEXT('B')
						  , TEXT('C'), TEXT('D'), TEXT('E'), TEXT('F')};
	TCHAR* exp = new TCHAR[l*2 + 1];
	const unsigned char* r = addr;
	const unsigned char* e = addr + l;
	TCHAR* w = exp;
	for (; r < e; r ++, w += 2)
	{
		unsigned char l = (*r & 0xf);
		unsigned char u = (*r >> 4);
		w[0] = code[l];
		w[1] = code[u];
	}
	*w = TEXT('\0');
	Logoutf(TEXT("%s:\n\t%s"), name, exp);
	delete [] exp;
}