/*
 * L86_printf.h
 *
 *  Created on: 17 abr. 2021
 *      Author: Santiago
 */

#ifndef L86_PRINTF_H_
#define L86_PRINTF_H_

/*====================[Includes]============================================*/
#include <string.h>
#include <stdint.h>
#include "printf.h"


/*====================[Typedefs enums and structs]==========================*/
/*
 * @brief	structure holding snprintf that format string as "$%s*%02X"
 * 			where %s is the command and %02X is the checksum
 */
typedef struct{
	char* buff;
	size_t idx;
	size_t maxlen;
	uint8_t checksum;
}printf_myfunc_t;


/*====================[Functions declarations]===============================*/
/*
 * @brief	snprintf formatting rcvBuff as "$%s*%02X" where %s according as fmt and data
 */
size_t L86_snprintf( char *rcvBuff, size_t maxLen, const char* fmt, ... );

/*
 * @brief	vsnprintf same as snprintf but receiving list instead data arg
 */
size_t L86_vsnprintf( char *rcvBuff, size_t maxLen, const char* fmt, va_list va );


#endif /* L86_PRINTF_H_ */
