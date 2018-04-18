#pragma once
#ifndef _UTILITY_H
#define _UTILITY_H
#include <tchar.h>
void __cdecl Logoutf(const TCHAR * _Format, ...);
void DumpBuffer(const TCHAR* name, const unsigned char* addr, unsigned int l);
#endif