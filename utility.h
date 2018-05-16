#pragma once
#ifndef _UTILITY_H
#define _UTILITY_H
#include <tchar.h>

#if !defined ASSERT
#ifdef _DEBUG
	#define ASSERT assert
#else
	#define ASSERT(exp) void(0)
#endif
#endif

void __cdecl Logoutf(const TCHAR * _Format, ...);
void DumpBuffer(const TCHAR* name, const unsigned char* addr, unsigned int l);
#endif