/*
 * L86_urc.c
 *
 *  Created on: 14 abr. 2021
 *      Author: Santiago
 */

/*====================[Includes]============================================*/
#include "L86.h"
#include "CLI.h"
#include "printf.h"

/*====================[Macros and defines]==================================*/
#define VALID		1U
#define INVALID		0U

#define TIME_PRINT_GNSS		pdMS_TO_TICKS( 5000 )

/*====================[Typedefs enums and structs]==========================*/

/*====================[Global declaration]===================================*/
// USART variables
extern usart_rtos_handle_t L86Uart;
usart_rtos_notification_t urcTaskNotification;

// gnssData variables
gnssData_t gnssData = { 0 };
#define cleanGnssData()			( memset( &gnssData, 0, sizeof(gnssData) ) )

// CLI variables
const char *L86Tok = "$,*";
CLI_Definition_t L86UrcCommands = {NULL};


/************************************************************************/
static const char cRMC[] = "RMC";
//static const char cVTG[] = "VTG";
static const char cGGA[] = "GGA";
//static const char cGSA[] = "GSA";
//static const char cGSV[] = "GSV";
//static const char cGLL[] = "GLL";
//static const char cTXT[] = "TXT";

//static const char *urc[] = {	cRMC,
//								cVTG,
//								cGGA,
//								cGSA,
//								cGSV,
//								cGLL
//								cTXT
//							};


/*====================[Private Functions]===================================*/
// NMEA convert functions
static void nmea2deg ( char* deg, size_t degLen, const char* nmea, const char* nmeaCP )
{
	if( NULL == deg || 0 == degLen )
		return;

	double minutes = strtod(nmea, NULL);
	double degree = (int) minutes / 100;
	minutes -= degree * 100;
	degree = degree + ( minutes / 60 );

	if( *nmeaCP == 'S' || *nmeaCP == 'W' )
		degree = degree * (-1);

	snprintf_(deg, degLen, "%.4f", degree);
}


static char* knots2kmh(char *Buff, size_t sizeBuff, const char *velKnots)
{
	if ( NULL == Buff || 0 == sizeBuff )
		return NULL;

	double velocidad = strtod(velKnots, NULL) * 1.852;

	if (velocidad < 0)
		strncpy(Buff, "0", sizeBuff);
	else
		snprintf_(Buff, sizeBuff, "%.2f", velocidad );

	return (Buff);
}


/*---------------------------------------------------------------------------------------*/
//CLI Functions
static BaseType_t rmcCommand( const char *pcCommandString, void *xUserData )
{
	PRINTF( "[RMC]->%s\n", pcCommandString );
	gnssData_t *gnssInfo = (gnssData_t*)xUserData;

	gnssInfo->rmcValid = INVALID;
	// Example of GGA Sentence
	// $GPRMC,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
	/* 		$ PG RMC ,							// 0
	 * 		(UTC)hhmmss.sss ,					// 1
	 * 		(Valid) V/A,						// 2
	 * 		(Latitude) ddmm.mmmm,				// 3
	 * 		N/S ,								// 4
	 * 		(Longitude) dddmm.mmmm ,			// 5
	 * 		E/W ,								// 6
	 * 		(speed) x.xx [knots],				// 7
	 * 		(COG) direction,					// 8
	 * 		(Date) ddmmyy,						// 9
	 * 		(magnetic),							// 10
	 * 		(magnetic) E/W,						// 11
	 * 		Position N/A/D,						// 12
	 * 		* (chks) */							// 13

	const char* _nextData = pcCommandString;
	size_t _lenData = strcspn(_nextData, ",*");
	int i;

	char coord[12] = {0};
	char coordCP[2] = {0};

	for( i = 0 , _nextData = pcCommandString ; *_nextData != '\0' && i < 14; i++, _lenData = strcspn(_nextData, ",*")  )
	{
		switch(i)
		{
		case 1:
			//_lenData
		{
			char hour[3] = {0};
			char min[3] = {0};
			char sec[8] = {0};
			strncpy(hour, _nextData, 2);
			strncpy(min, _nextData+2, 2);
			strncpy(sec, _nextData+4, _lenData - 4);

			snprintf_( gnssInfo->hour, sizeof( gnssInfo->hour), "%s:%s:%s", hour, min, sec );
		}
			break;
		case 3:
			strncpy( coord, _nextData, _lenData );
			coord[sizeof(coord) - 1] = '\0';			// no null character implicity aooended at the end of destination if source is longer than num
			break;
		case 4:
			strncpy( coordCP, _nextData, _lenData );
			nmea2deg( gnssInfo->latitude , sizeof(gnssInfo->latitude), coord, coordCP);
			break;
		case 5:
			strncpy( coord, _nextData, _lenData );
			coord[sizeof(coord) - 1] = '\0';			// no null character implicity aooended at the end of destination if source is longer than num
			break;
		case 6:
			strncpy( coordCP, _nextData, _lenData );
			nmea2deg( gnssInfo->longitude , sizeof(gnssInfo->longitude), coord, coordCP);
			break;
		case 7:
			strncpy( gnssInfo->speed, _nextData, _lenData );
			knots2kmh( gnssInfo->speed, sizeof(gnssInfo->speed), gnssInfo->speed);
			break;
		case 8:
			strncpy( gnssInfo->direction, _nextData, _lenData );
			break;
		case 9:
		{
			char day[3] = {0};
			char month[3] = {0};
			char year[3] = {0};
			strncpy(day, _nextData, 2);
			strncpy(month, _nextData+2, 2);
			strncpy(year, _nextData+4, 2);
			snprintf_( gnssInfo->date, sizeof( gnssInfo->hour), "%s/%s/%s", day, month, year );
			gnssInfo->rmcValid = VALID;
			break;
		}
		default:
			break;
		}

		_nextData += _lenData + 1;
	}

	return gnssInfo->rmcValid == VALID ? pdTRUE : pdFALSE;
}


static BaseType_t ggaCommand( const char *pcCommandString, void *xUserData )
{
	PRINTF( "[GGA]->%s\n", pcCommandString );
	gnssData_t *gnssInfo = (gnssData_t*)xUserData;

	gnssInfo->ggaValid = INVALID;
//	char* ggaDataToGnss[] = { gnssData.state, gnssData.satellites, gnssData.Height };

	// Example of GGA Sentence
	// $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
	/* 		$ PG GGA ,							// 0
	 * 		(UTC)hhmmss.sss ,					// 1
	 * 		(Latitude) ddmm.mmmm ,				// 2
	 * 		N/S ,								// 3
	 * 		(Longitude) dddmm.mmmm ,			// 4
	 * 		E/W ,								// 5
	 * 		(Fix) 0/1/2/6 ,						// 6
	 * 		(satellites) 0:24 ,					// 7
	 * 		(HDOP) x.xx ,						// 8
	 * 		(Altitude) x.xx ,					// 9
	 * 		M ,									// 10
	 * 		(Geoid) ,							// 11
	 * 		M ,									// 12
	 * 		(DGPS Age) ? ,						// 13
	 * 		(DGPS Id) ? ,						// 14
	 * 		* (chks) */							// 15

	const char* _nextData = pcCommandString;
	size_t _lenData = strcspn(_nextData, ",*");
	int i;
	for( i = 0 , _nextData = pcCommandString ; *_nextData != '\0' && i < 16; i++, _lenData = strcspn(_nextData, ",*")  )
	{
		switch(i)
		{
			case 6:
				strncpy( gnssInfo->state, _nextData, _lenData );
				break;
			case 7:
				strncpy( gnssInfo->satellites, _nextData, _lenData );
				break;
			case 9:
				strncpy( gnssInfo->Height, _nextData, _lenData );
				gnssInfo->ggaValid = VALID;
				break;
		}

		_nextData += _lenData + 1;
	}

	gnssInfo->allDataProcessed = pdTRUE;
	return gnssInfo->ggaValid == VALID ? pdTRUE : pdFALSE;
}


/*====================[CLI commands definitions]===================================*/
static const CLI_Command_Definition_t urcRMC =
{
	cRMC,
	rmcCommand,
	-1
};

//const CLI_Command_Definition_t urcVTG =
//{
//	cVTG,
//	vtgCommand,
//	0
//};

static const CLI_Command_Definition_t urcGGA =
{
	cGGA,
	ggaCommand,
	-1
};

//const CLI_Command_Definition_t urcGSA =
//{
//	"GSA",
//	gsaCommand,
//	0
//};

//const CLI_Command_Definition_t urcGSV =
//{
//	"GSV",
//	gsvCommand,
//	0
//};

//const CLI_Command_Definition_t urcGLL =
//{
//	"GLL",
//	gllCommand,
//	0
//};

//const CLI_Command_Definition_t urcGPTXT =
//{
//	"TXT",
//	gptxtCommand,
//	0
//};


/*====================[Functions definitions]===============================*/
//CLI
static void registerCommands( void )
{
	L86UrcCommands.commandList = NULL;//&L86DefinitionLists;
	L86UrcCommands.tokens = (char*)L86Tok;

	CLI_RegisterCommand( &L86UrcCommands, &urcRMC);
	CLI_RegisterCommand( &L86UrcCommands, &urcGGA);
}

/*====================[Tasks]==============================================*/

void L86_urc_task(void *pvParameters)
{
	uint8_t _urcBuff[URC_BUFF_SIZE];
	char* _buff;
	TimeOut_t _timeout;
	TickType_t _timeBetweenPrints = TIME_PRINT_GNSS;

	registerCommands();

	urcTaskNotification.transfer.data = _urcBuff;
	urcTaskNotification.transfer.dataSize = sizeof(_urcBuff);
	urcTaskNotification.task = xTaskGetCurrentTaskHandle();
	USART_RTOS_SetURC( &L86Uart, &urcTaskNotification );

	vTaskSetTimeOutState( &_timeout );

	while(1)
	{
		_buff = (char*)_urcBuff;
		// wait for incoming string
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

//		if( pdPASS == ChkSum_Verify( (uint8_t*)_buff, strlen(_buff) ) )
		if( pdPASS == ChkSum_Verify( (uint8_t*)_buff ) )
		{
			// TODO check if GP GN or GL
			_buff += 3;		// advance $-- where -- is Talker ID GP/GN when RMC/GSA/GLL OR GP/GL when GSV otherwise GP
			CLI_ProcessCommand( &L86UrcCommands, _buff, &gnssData );
		}
		USART_RTOS_giveFromUrc( &L86Uart );


		if( 1 == gnssData.allDataProcessed )
		{
			if( gnssData.rmcValid == VALID && gnssData.ggaValid == VALID )
			{
				if( xTaskCheckForTimeOut( &_timeout, &_timeBetweenPrints ) == pdTRUE)
				{
					vTaskSetTimeOutState( &_timeout );
					if( gnssData.state[0] != '1' && gnssData.state[0] != '2' )
					{
						PRINTF( "Not GNSS Signal\n" );
					}
					else
					{
						PRINTF( "Date: %s %s\n"
								"Latitude: %s\n"
								"Longitude: %s\n"
								"Height: %s\n"
								"Speed: %s\n"
								"Angle: %s\n"
								"Satellites: %s\n\n",
								gnssData.date , gnssData.hour ,
								gnssData.latitude , //gnssData.latitudeCP ,
								gnssData.longitude , //gnssData.longitudeCP ,
								gnssData.Height,
								gnssData.speed,
								gnssData.direction,
								gnssData.satellites
								);
					}
				}
			}
			cleanGnssData();
		}
	}
}
