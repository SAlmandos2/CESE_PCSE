/*
 * L86.c
 *
 *  Created on: 14 abr. 2021
 *      Author: Santiago
 */

/*====================[Includes]============================================*/
#include <string.h>
#include "printf.h"
#include "fsl_iocon.h"
#include "pin_mux.h"
#include "L86.h"
#include "L86_printf.h"

/*====================[Macros and defines]==================================*/
#define L86_PRINTF_BUF_SIZE		86
//#define L86_PRINTF_TIMEOUT		pdMS_TO_TICKS( 50 )			// define on trial and error
#define L86_PRINTF_TIMEOUT		pdMS_TO_TICKS( 10000 )

#define L86_CMD_TIMEOUT			pdMS_TO_TICKS( 1000 )			// 1s

/*====================[Typedefs enums and structs]==========================*/

static uint32_t	L86_baudRate[] = {
		[L86_Baud_4800] 	=  4800,
		[L86_Baud_9600] 	=  9600,
		[L86_Baud_14400] 	=  14400,
		[L86_Baud_19200] 	=  19200,
		[L86_Baud_38400] 	=  38400,
		[L86_Baud_57600] 	=  57600,
		[L86_Baud_115200]	=  115200
};

/*====================[Global declaration]===================================*/
usart_rtos_handle_t L86Uart;


/*====================[Private Functions]===================================*/
static void L86_pinInit( void );


/*====================[Extern Functions]====================================*/
extern void L86_urc_task(void *pvParameters);


/*====================[Functions definitions]===============================*/
uint8_t ChkSum_Calc ( uint8_t *start, size_t length )
{
	if( 0 == length || NULL == start )
		return 0;

	int i;
	uint32_t chks = 0;
	for( i = 0 ; i < length ; i++ )
	{
		chks ^= start[i];
	}

	return (uint8_t)(chks & 0xFFUL);
}


//BaseType_t ChkSum_Verify ( uint8_t *start, size_t length )
BaseType_t ChkSum_Verify ( uint8_t *start )
{
//	if( 0 == length || NULL == start )
	if( NULL == start )
		return pdFAIL;

	uint8_t* _toCheck = start;
	char* _chksSentence;
	char _chks[3] = { 0 };
//	uint8_t _chksCalc;

	_chksSentence = strrchr( (char*)_toCheck, '*' );

	if( *_toCheck == '$' && _chksSentence != NULL )
	{
		_toCheck++;
		snprintf_( _chks, sizeof(_chks), "%02X", ChkSum_Calc( _toCheck, (uint8_t*)_chksSentence - _toCheck ) );
		if( memcmp( _chks, _chksSentence + 1, 2 ) == 0 )
			return pdPASS;
	}
	return pdFAIL;
}

/*====================[L86 specific functions]==============================*/

static void L86_pinInit ( )
{
    const uint32_t port1_pin10_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
										 IOCON_FUNC2 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 1U, 10U, port1_pin10_config);

    const uint32_t port1_pin11_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
    									 IOCON_FUNC2 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 1U, 11U, port1_pin11_config);
}


BaseType_t L86_Init ( l86_usart_config config )
{
	L86_pinInit();

	if ( kStatus_Success != USART_RTOS_Init( &L86Uart, &config ) )
	{
		PRINTF( "L86 couldn't init" );
		return pdFAIL;
	}

	if( xTaskCreate(	L86_urc_task,
						"L86_urc",
						configMINIMAL_STACK_SIZE + URC_BUFF_SIZE,
						NULL,
						uart_task_PRIORITY - 1,
						NULL) != pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		USART_RTOS_Deinit( &L86Uart );
		return pdFAIL;
	}

	return pdPASS;
}


void L86_Deinit ( void )
{
	 USART_RTOS_Deinit( &L86Uart );
}

BaseType_t L86_Cmd( TickType_t timeout, const char* fmt, ... )
{
	status_t _status;
	TimeOut_t _timeout;

	va_list va;
	va_start(va, fmt);

	char _internalBuff[ L86_PRINTF_BUF_SIZE ] = {0};
	L86_vsnprintf(_internalBuff, L86_PRINTF_BUF_SIZE, fmt, va);

	vTaskSetTimeOutState( &_timeout );
	if( ( _status = xSemaphoreTake( L86Uart.semaphore, timeout ) ) == pdPASS )
	{
		xTaskCheckForTimeOut( &_timeout, &timeout );

		_status = USART_RTOS_Send( &L86Uart, (uint8_t*)_internalBuff, timeout );
		(void)xSemaphoreGive( L86Uart.semaphore );
	}

	va_end(va);
	return ( kStatus_Success == _status ? pdPASS : pdFAIL );
}


BaseType_t L86_CmdResponse(char* rcv, uint32_t rcvSize, TickType_t timeout, const char* fmt, ... )
{
	va_list va;
	va_start(va, fmt);

	uint32_t result = pdFAIL;

	status_t _status;
	char _internalBuff[ L86_PRINTF_BUF_SIZE ] = {0};

	L86_vsnprintf(_internalBuff, L86_PRINTF_BUF_SIZE, fmt, va);

	_status = USART_RTOS_Send_Receive( &L86Uart, (uint8_t*)_internalBuff, (uint8_t*)rcv, rcvSize, timeout );

	if( kStatus_Success == _status)
		result = ChkSum_Verify( (uint8_t*)rcv);
//		result = ChkSum_Verify( (uint8_t*)rcv, strlen(rcv) );

	va_end(va);
	return result;
}


BaseType_t L86_CmdOK( TickType_t timeout, const char* fmt, ... )
{
	va_list va;
	va_start(va, fmt);

	uint32_t _result = pdFAIL;

	status_t _status;
	char _internalBuffSend[ L86_PRINTF_BUF_SIZE ] = {0};
	char _internalBuffResponse[ L86_PRINTF_BUF_SIZE ] = {0};

	L86_vsnprintf(_internalBuffSend, sizeof(_internalBuffSend), fmt, va);

	_status = USART_RTOS_Send_Receive( &L86Uart, (uint8_t*)_internalBuffSend, (uint8_t*)_internalBuffResponse, sizeof(_internalBuffResponse), timeout );

	if( kStatus_Success == _status)
	{
//		if( pdPASS == ChkSum_Verify( (uint8_t*)_internalBuffResponse, strlen(_internalBuffResponse) ) )
		if( pdPASS == ChkSum_Verify( (uint8_t*)_internalBuffResponse ) )
		{
			if( strstr( _internalBuffResponse , "OK" ) != NULL )
				_result = pdPASS;
			else if( strstr( _internalBuffResponse , "ERROR" ) != NULL )
				_result = pdFAIL;
		}
	}
	va_end(va);
	return _result;
}


BaseType_t L86_CmdPmtkOK( TickType_t timeout, const char* cmd, const char* fmt, ... )
{
	va_list va;
	va_start(va, fmt);

	uint32_t _result = pdFAIL;

	status_t _status;
	char _format[L86_PRINTF_BUF_SIZE];
	char _internalBuff[ L86_PRINTF_BUF_SIZE ] = {0};

	snprintf_( _format, sizeof(_format), "PMTK%s,%s", cmd, fmt );

	L86_vsnprintf( _internalBuff, sizeof(_internalBuff), _format, va );

	_status = USART_RTOS_Send_Receive( &L86Uart, (uint8_t*)_internalBuff, (uint8_t*)_internalBuff, sizeof(_internalBuff), timeout );

	if( kStatus_Success == _status)
	{
//		if( pdPASS == ChkSum_Verify( (uint8_t*)_internalBuff, strlen(_internalBuff) ) )
		if( pdPASS == ChkSum_Verify( (uint8_t*)_internalBuff ) )
		{
			char* _internalBuffPointer = _internalBuff + 1;	//ignore $
			size_t _len = strcspn( _internalBuffPointer, ",*" );

			if( memcmp ( _internalBuffPointer, "PMTK001", _len ) == 0)
			{
				_internalBuffPointer += (_len + 1);
				_len = strcspn( _internalBuffPointer, ",*" );
				if( _len == 3 && memcmp( _internalBuffPointer , cmd, _len ) == 0)		//sizeof cmd should be always 3
				{
					_internalBuffPointer += (_len + 1);
					_len = strcspn( _internalBuffPointer, ",*" );
					/*	0 = Invalid packet
					 * 	1 = Unsupported packet type
					 * 	2 = Valid packet, but action failed
					 * 	3 = Valid packet, action succeeded
					 */
					if( *_internalBuffPointer == '3' )
						_result = pdTRUE;
				}
			}
		}
	}
	va_end(va);
	return _result;
}


/*====================[Functions commands]=================================*/
BaseType_t L86_changeBaud( L86_baudrate_t baud )
{
//	char _internalBuff[ L86_PRINTF_BUF_SIZE ] = {0};

	if( baud > L86_Baud_115200 )
		return pdFAIL;

	return L86_Cmd( L86_CMD_TIMEOUT, "PQBAUD,W,%i", L86_baudRate[baud] );
}


BaseType_t L86_getOutput( uint32_t *output )
{
	char _internalBuff[L86_PRINTF_BUF_SIZE] = {0};

	uint32_t _output = 0;
	if( pdPASS == L86_CmdResponse( _internalBuff, L86_PRINTF_BUF_SIZE, L86_CMD_TIMEOUT, "PMTK414" ) )
	{
		int i;
		char* _internalBuffPointer = _internalBuff + 1;	//ignore $
		size_t _len = strcspn( _internalBuffPointer, ",*" );

		for( i = 0 , _internalBuffPointer = _internalBuff + 1 ; *_internalBuffPointer != '\0' && i < 24; i++, _len = strcspn(_internalBuffPointer, ",*")  )
		{
			if( ( i > 0 ) && ( i < 23 ) )
			{
				_output |= ( *_internalBuffPointer == '1' ? 1 : 0 ) << (i - 1);
			}

			_internalBuffPointer += (_len + 1);
		}

		*output = ( _output & NMEA_OUTPUT_MASK );
		return pdPASS;
	}
	return pdFAIL;
}


BaseType_t L86_setOutput( int32_t output )
{
	BaseType_t _result;
	if( DEFAULT_OUTPUT == output )
		_result = L86_CmdPmtkOK( L86_CMD_TIMEOUT, "314" , "%i", output );
	else
		_result = L86_CmdPmtkOK( 	L86_CMD_TIMEOUT,
								"314" ,
								"%i,%i,%i,%i,%i,%i,%i,%i,0,0,0,0,0,0,0,0,0,%i,%i",
								IS_NMEA_ENABLED( GLL_OUTPUT, output ),
								IS_NMEA_ENABLED( RMC_OUTPUT, output ),
								IS_NMEA_ENABLED( VTG_OUTPUT, output ),
								IS_NMEA_ENABLED( GGA_OUTPUT, output ),
								IS_NMEA_ENABLED( GSA_OUTPUT, output ),
								IS_NMEA_ENABLED( GSV_OUTPUT, output ),
								IS_NMEA_ENABLED( GRS_OUTPUT, output ),
								IS_NMEA_ENABLED( GST_OUTPUT, output ),
								IS_NMEA_ENABLED( ZDA_OUTPUT, output ),
								IS_NMEA_ENABLED( MCHN_OUTPUT, output )
							);
	return _result;
}
