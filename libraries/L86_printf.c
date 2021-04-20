/*
 * L86_printf.c
 *
 *  Created on: 17 abr. 2021
 *      Author: Santiago
 */


#include "L86_printf.h"
#include <stdarg.h>


static void snprintf_chks( char character, void* arg )
{
	printf_myfunc_t *myFunc = arg;

	if (myFunc->idx < myFunc->maxlen) {
	((char*)myFunc->buff)[myFunc->idx] = character;
	}

	if( character == '$')		//ignore this characters
	{}	//ignore start char to calculate check sum
	else if (character == '*')
	{}	//ignore end char to calculate check sum
	else
		myFunc->checksum ^= (uint8_t)character;

	myFunc->idx++;
}


size_t L86_snprintf( char *rcvBuff, size_t maxLen, const char* fmt, ... )
{
	printf_myfunc_t L86printfFunc = {0};
	size_t ret;
	va_list va;
	va_start(va, fmt);
	L86printfFunc.buff = rcvBuff;
	L86printfFunc.maxlen = maxLen;

	ret = fctprintf( &snprintf_chks, &L86printfFunc, "$" );
	ret += vfctprintf( &snprintf_chks, &L86printfFunc, fmt, va );
	ret += fctprintf( &snprintf_chks, &L86printfFunc, "*%02X\r\n", L86printfFunc.checksum );
	va_end(va);

	return ret;
}

size_t L86_vsnprintf( char *rcvBuff, size_t maxLen, const char* fmt, va_list va )
{
	printf_myfunc_t L86printfFunc = {0};
	size_t ret;

	L86printfFunc.buff = rcvBuff;
	L86printfFunc.maxlen = maxLen;

	ret = fctprintf( &snprintf_chks, &L86printfFunc, "$" );
	ret += vfctprintf( &snprintf_chks, &L86printfFunc, fmt, va );
	ret += fctprintf( &snprintf_chks, &L86printfFunc, "*%02X\r\n", L86printfFunc.checksum );

	return ret;
}
