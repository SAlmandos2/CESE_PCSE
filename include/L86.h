/*
 * L86.h
 *
 *  Created on: 14 abr. 2021
 *      Author: Santiago
 */

#ifndef L86_H_
#define L86_H_

/*====================[Includes]============================================*/
#include "usart_rtos.h"
#include "FreeRTOS.h"
#include "semphr.h"


/*====================[Macros and defines]==================================*/
#define L86_DEFAULT_TIMEOUT		pdMS_TO_TICKS( 1000 )

#define uart_task_PRIORITY (configMAX_PRIORITIES - 2)

#define BACKBUFF_SIZE	16
static uint8_t backBuff[BACKBUFF_SIZE];

#define URC_BUFF_SIZE	128

/* to change nmea output */
#define DEFAULT_OUTPUT						( -1L )
#define GLL_OUTPUT							( 1L << 0 )
#define RMC_OUTPUT							( 1L << 1 )
#define VTG_OUTPUT							( 1L << 2 )
#define GGA_OUTPUT							( 1L << 3 )
#define GSA_OUTPUT							( 1L << 4 )
#define GSV_OUTPUT							( 1L << 5 )
#define GRS_OUTPUT							( 1L << 6 )
#define GST_OUTPUT							( 1L << 7 )
#define ZDA_OUTPUT							( 1L << 17 )
#define MCHN_OUTPUT							( 1L << 18 )
#define IS_NMEA_ENABLED( nmea, output )		( ( (int32_t)nmea & (int32_t)output ) ? 1 : 0 )
#define NMEA_OUTPUT_MASK					( 0x60FF )

/*====================[Typedefs enums and structs]==========================*/

typedef usart_rtos_config_t		l86_usart_config;

/* Edit this struct if you want to modify boot USART configuration */
static const usart_rtos_config_t L86UartConfig = {
	.base 			= 	USART1,
	.baudrate		= 	9600,
	.parity			= 	kUSART_ParityDisabled,
	.stopbits    	= 	kUSART_OneStopBit,
	.buffer      	= 	backBuff,
	.buffer_size 	= 	BACKBUFF_SIZE,
};


/* baudrate for command L86_changeBaud */
typedef enum{
	L86_Baud_4800,
	L86_Baud_9600,
	L86_Baud_14400,
	L86_Baud_19200,
	L86_Baud_38400,
	L86_Baud_57600,
	L86_Baud_115200
}L86_baudrate_t;

/* structure holding GNSS data */
typedef struct{
//	uint32_t gllValid : 1;
	uint32_t rmcValid : 1;
//	uint32_t vtgValid : 1;
	uint32_t ggaValid : 1;
//	uint32_t gsaValid : 1;
//	uint32_t gsvValid : 1;
//	uint32_t grsValid : 1;
//	uint32_t gstValid : 1;
//	uint32_t zdaValid : 1;
//	uint32_t mchnValid : 1;
//	uint32_t txtValid : 1;
	uint32_t allDataProcessed : 1;			// change to 1 when last sentence arrives
	// RMC
	char hour[15];
	char latitude[15];
//	char latitudeCP[15];
	char longitude[15];
//	char longitudeCP[15];
	char speed[15];
	char direction[15];
	char date[15];
	// GGA
	char state[2];
	char satellites[3];
	char Height[10];
}gnssData_t;

//
//typedef enum{
//	PQBAUD,
//	PQEPE,
//	PQ1PPS,
//	PQFLP,
//	PQTXT,
//	PQEXEF,
//	PQODO,
//	PQPZ90,
//	PQGLP,
//	PQVEL,
//	PQJAM,
//	PQRLM,
//	PQGEO,
//	PQPREC,
//	PQGBS,
//}pqCommands_t;

/*	@brief: This commands are for SDK Quectel L86
 *  		All start with "PQ" and then the comand below
 *  @Ref:	"GNSS_SDK_Commands_Manual_V1.4"
 */
//const char *cmnds_Quectel[] = {
//								[PQBAUD]	=	"BAUD",
//								[PQEPE]		=	"EPE",
//								[PQ1PPS]	=	"1PPS",
//								[PQFLP]		=	"FLP",
//								[PQTXT]		=	"TXT",
//								[PQEXEF]	=	"EXEF",
//								[PQODO]		=	"ODO",
//								[PQPZ90]	=	"PZ90",
//								[PQGLP]		=	"GLP",
//								[PQVEL]		=	"VEL",
//								[PQJAM]		=	"JAM",
//								[PQRLM]		=	"RLM",
//								[PQGEO]		=	"GEO",
//								[PQPREC]	=	"PREC",
//								[PQGBS]		=	"GBS"
//								};

//typedef enum{
//	PMTK_SYS_MSG = 10,
//	PMTK_TXT_MSG,
//	PMTK_ACK,
//	PMTK_CMD_HOT_START,
//	PMTK_CMD_WARM_START,
//	PMTK_CMD_COLD_START,
//	PMTK_CMD_FULL_COLD_START,
//	PMTK_CMD_STANDBY_MODE,
//	PMTK_LOCUS_ERASE_FLASH,
//	PMTK_LOCUS_STOP_LOGGER,
//	PMTK_Q_LOCUS_DATA,
//	PMTK_SET_POS_FIX,
//	PMTK_SET_AL_DEE_CFG,
//	PMTK_SET_PERIODIC_MODE,
//
//}pmtpkCommands_t;

/*====================[Functions definitions]===============================*/
/* Checksum calculation and verification */
/*
 * @brief	Calculate 8 bits checksum from start, length bytes.
 * @param	start: pointer to string to calculate checksum
 * @param	length: length of string to calculate checksum
 * @return	calculated 8bit checksum
 */
uint8_t ChkSum_Calc ( uint8_t *start, size_t length );

/*
 * @brief	Receive an pointer to a nmea string, calculate the
 * 			checksum and verify it with the one  received
 * @param	start: pointer to nmea string
 */
BaseType_t ChkSum_Verify ( uint8_t *start );
//BaseType_t ChkSum_Verify ( uint8_t *start, size_t length );

/*
 * @brief	Initialize L86 usart with configuration received
 * @param	config: struct that holds the configuration of usart.
 * @return	pdPASS if usart initialization succedded, pdFAIL if
 * 			coudnt initialize usart
 */
BaseType_t L86_Init ( l86_usart_config config );

/*
 * @brief	Deinitialize usart used by L86
 */
void L86_Deinit ( void );

/*
 * @brief	Send command to L86. No waits for reception
 * @param	timeout: ticks to wait before cancel send and return
 * @param	fmt: format as printf. Other parametters received acording to fmt
 * @return	pdPASS if command send, pdFAIL if coudnt send it
 */
BaseType_t L86_Cmd( TickType_t timeout, const char* fmt, ... );

/*
 * @brief	Send command and wait for response. Received response checksum is verified
 * @param	rcv: pointer to buffer where string received will be stored
 * @param	rcvSize: max size of rcv buffer
 * @param	timeout: ticks to wait before cancel send and reception
 * @param	fmt: format as printf. Other parameters received according to fmt
 * @return	pdPASS if response was received ok, pdFAIL if coudn't send or receive
 */
BaseType_t L86_CmdResponse(char* rcv, uint32_t rcvSize, TickType_t timeout, const char* fmt, ... );

/*
 * @brief	Send command and wait for "OK response"
 * @param 	timeout: ticks to wait before cancel send and reception
 * @param	fmt: format as printf. Other parameters received according to fmt
 * @return	pdPASS if OK was received, pdFAIL if no response or ERROR received
 */
BaseType_t L86_CmdOK(TickType_t timeout, const char* fmt, ... );

/*--------------------------------------------------------------------------*/
/*
 * @brief	Send command to change baudrate of L86 module. no response will be received,
 * 			and baudrate will be changed immediately
 * @param	baud: baudrate selected to change, from L86_baudrate_t
 * @return	pdPASS if command send, pdFAIL if coudn't send it
 */
BaseType_t L86_changeBaud( L86_baudrate_t baud );

/*
 * @brief	Get nmea sentences output from uart module
 * @param	output: pointer to variable that will hold the output configuration
 * @return
 */
BaseType_t L86_getOutput( uint32_t *output );

/*
 * @brief	This function set the nmea sentence output.
 * 			use XXX_OUTPUT to select output. Can be ORed to select multiples
 * 			example $PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29<CR><LF>
 * 							GLL , RMC, VTG, GGA, GSA, GSV, GRS, GST, Reserved, Reserved, Reserved, Reserved, Reserved, Reserved, Reserved, Reserved, Reserved,  ZDA, MCHN * chksum
 * 			default $PMTK314,-1*04<CR><LF>
 * @param	output:	integer with selected sentences to configure.
 * @return	pdPASS if succedded or pdFAIL if not.
 */
BaseType_t L86_setOutput( int32_t output );


#endif /* L86_H_ */
