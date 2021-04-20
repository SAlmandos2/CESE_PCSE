/*
 * uart_rtos.c
 *
 *  Created on: 22 mar. 2021
 *      Author: Santiago
 */

#include "usart_rtos.h"
#include "fsl_reset.h"
#include "fsl_flexcomm.h"

/*==========[Definitions and Macros]===================================================================================================================*/
#define USART_RTOS_EVENT_INDEX	0
//#define USART_RTOS_HANDLER_INDEX	1
//#define RECEIVER_TIMOUT_ERROR	pdMS_TO_TICKS( 60000U )		//set this to never occur except a USART connection fail
#define RECEIVER_TIMOUT_ERROR	portMAX_DELAY		//set this to never occur except a USART connection fail

#define USART_NVIC_PRIO		5

/*==========[Types and enums]==========================================================================================================================*/


/*==========[variables definitions]===================================================================================================================*/
static const IRQn_Type IRQ_Flexcom[8] = { FLEXCOMM0_IRQn, FLEXCOMM1_IRQn, FLEXCOMM2_IRQn, FLEXCOMM3_IRQn,
							 	 	 	  FLEXCOMM4_IRQn, FLEXCOMM5_IRQn, FLEXCOMM6_IRQn, FLEXCOMM7_IRQn};

static const clock_attach_id_t clock_attach[8][2] = {
													{ kFRO12M_to_FLEXCOMM0, kOSC32K_to_FLEXCOMM0 },
													{ kFRO12M_to_FLEXCOMM1, kOSC32K_to_FLEXCOMM1 },
													{ kFRO12M_to_FLEXCOMM2, kOSC32K_to_FLEXCOMM2 },
													{ kFRO12M_to_FLEXCOMM3, kOSC32K_to_FLEXCOMM3 },
													{ kFRO12M_to_FLEXCOMM4, kOSC32K_to_FLEXCOMM4 },
													{ kFRO12M_to_FLEXCOMM5, kOSC32K_to_FLEXCOMM5 },
													{ kFRO12M_to_FLEXCOMM6, kOSC32K_to_FLEXCOMM6 },
													{ kFRO12M_to_FLEXCOMM7, kOSC32K_to_FLEXCOMM7 },
													};

static const reset_ip_name_t rst_names[] = FLEXCOMM_RSTS;

/**/

static void USART_RTOS_ReceiveTask( void* param );

/*==========[Function callbacks]=====================================================================================================================*/
static void USART_RTOS_Callback(USART_Type *base, usart_handle_t *state, status_t status, void *param)
{
	usart_rtos_handle_t *handle = (usart_rtos_handle_t *)param;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFAIL;

//    xHigherPriorityTaskWoken = pdFALSE;
//    xResult                  = pdFAIL;

    if (status == kStatus_USART_RxIdle)
    {
    	xResult = xTaskNotifyFromISR( handle->ReceiveHandler, RTOS_USART_COMPLETE, eSetBits, &xHigherPriorityTaskWoken );
//        xResult = xEventGroupSetBitsFromISR(handle->rxEvent, RTOS_USART_COMPLETE, &xHigherPriorityTaskWoken);
    }
    else if (status == kStatus_USART_TxIdle)
    {
    	configASSERT(handle->SenderHandler);
    	xResult = xTaskNotifyFromISR( handle->SenderHandler, RTOS_USART_COMPLETE, eSetBits, &xHigherPriorityTaskWoken );
//        xResult = xEventGroupSetBitsFromISR(handle->txEvent, RTOS_USART_COMPLETE, &xHigherPriorityTaskWoken);
    }
    else if (status == kStatus_USART_RxRingBufferOverrun)
    {
    	xResult = xTaskNotifyFromISR( handle->ReceiveHandler, RTOS_USART_RING_BUFFER_OVERRUN, eSetBits, &xHigherPriorityTaskWoken );
//        xResult = xEventGroupSetBitsFromISR(handle->rxEvent, RTOS_USART_RING_BUFFER_OVERRUN, &xHigherPriorityTaskWoken);
    }
    else
    {
        xResult = pdFAIL;
    }

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


static uint8_t wichUsartUsed( usart_rtos_handle_t* handle )
{
	uint8_t usart;
	switch( (uint32_t)handle->base )
	{
		case USART0_BASE:
			usart = 0;
			break;
		case USART1_BASE:
			usart = 1;
			break;
		case USART2_BASE:
			usart = 2;
			break;
		case USART3_BASE:
			usart = 3;
			break;
		case USART4_BASE:
			usart = 4;
			break;
		case USART5_BASE:
			usart = 5;
			break;
		case USART6_BASE:
			usart = 6;
			break;
		case USART7_BASE:
			usart = 7;
			break;
	}
	return usart;
}

/*==========[Public functions]======================================================================================================================*/
int USART_RTOS_Init( usart_rtos_handle_t* handle, const usart_rtos_config_t *config)
{
	usart_config_t _usart_config;
	uint32_t instance = FLEXCOMM_GetInstance(config->base);
	status_t _status;

	if( NULL == handle || NULL == config->base || 0 == config->baudrate )
		return kStatus_InvalidArgument;

    RESET_ClearPeripheralReset(rst_names[instance]);

	handle->base = config->base;

	handle->semaphore = xSemaphoreCreateMutex();
	if (NULL == handle->semaphore)
	{
		return kStatus_Fail;
	}

	USART_GetDefaultConfig(&_usart_config);

	if( config->baudrate < 9600 )
	{
		CLOCK_AttachClk(clock_attach[instance][1]);
		_usart_config.enableMode32k = true;
	}
	else
		CLOCK_AttachClk(clock_attach[instance][0]);

	_usart_config.baudRate_Bps = config->baudrate;
	_usart_config.enableTx     = true;
	_usart_config.enableRx     = true;
	_usart_config.parityMode = config->parity;
	_usart_config.stopBitCount = config->stopbits;

	NVIC_SetPriority(IRQ_Flexcom[instance], USART_NVIC_PRIO);

	_status = USART_Init( (USART_Type*)handle->base, &_usart_config, CLOCK_GetFlexCommClkFreq(instance) );
	if (_status != kStatus_Success)
	{
		vSemaphoreDelete(handle->semaphore);
		return kStatus_Fail;
	}

	/* Task should be created before callback to prevent task notification to NULL handler */
	TaskHandle_t rcvTask;
	if( xTaskCreate( USART_RTOS_ReceiveTask, "rcv_task", configMINIMAL_STACK_SIZE + 100, handle, configMAX_PRIORITIES - 1 , &rcvTask) != pdPASS )
	{
		PRINTF("Task creation failed!.\r\n");
		vSemaphoreDelete(handle->semaphore);
		return kStatus_Fail;
	}
	handle->ReceiveHandler = rcvTask;

	_status = USART_TransferCreateHandle( (USART_Type*)handle->base, &(handle->usart_handle), USART_RTOS_Callback, handle );
	if (_status != kStatus_Success)
	{
		vSemaphoreDelete(handle->semaphore);
		vTaskDelete(rcvTask);
		return kStatus_Fail;
	}

	USART_TransferStartRingBuffer( (USART_Type*)handle->base, &(handle->usart_handle), config->buffer, config->buffer_size );

	handle->txTransfer.data = 0;
	handle->txTransfer.dataSize = 0;
	handle->rxTransfer.data = 0;
	handle->rxTransfer.dataSize = 0;

	handle->SenderHandler = (TaskHandle_t)NULL;
	handle->URC_handler_notification = (usart_rtos_notification_t*)NULL;
	handle->CMD_handler_notification = (usart_rtos_notification_t*)NULL;

	handle->status = USART_RTOS_IDLE;

	vTaskResume( rcvTask );

	return kStatus_Success;
}


void USART_RTOS_Deinit(usart_rtos_handle_t *handle)
{
	USART_Deinit( (USART_Type*)handle->base );

	/* Give the semaphore. This is for functional safety */
	(void)xSemaphoreGive(handle->semaphore);

	vSemaphoreDelete(handle->semaphore);

	vTaskDelete(handle->ReceiveHandler);
	handle->ReceiveHandler = NULL;

	handle->base = NULL;
//	handle->usart_handle = NULL;
}


void USART_RTOS_Reset(usart_rtos_handle_t *handle)
{
	taskENTER_CRITICAL();
//	USART_TransferAbortReceive( (USART_Type*)_handle->base, &(_handle->usart_handle) );

	handle->txTransfer.data = NULL;
	handle->txTransfer.dataSize = 0;
	handle->rxTransfer.data = NULL;
	handle->rxTransfer.dataSize = 0;
	handle->status = USART_RTOS_IDLE;
	xSemaphoreGive(handle->semaphore);
	xTaskNotify( handle->ReceiveHandler, RTOS_USART_RESET, eSetBits );
	taskEXIT_CRITICAL();
}

status_t USART_RTOS_SetURC(  usart_rtos_handle_t *handle, usart_rtos_notification_t *taskNotification )
{
	if( NULL == handle || NULL == taskNotification )
			return kStatus_InvalidArgument;

	handle->URC_handler_notification = taskNotification;
	return kStatus_Success;
}


status_t USART_RTOS_Send( usart_rtos_handle_t *handle, const uint8_t* toSend, size_t sizeToSend, TickType_t timeout )
{
	BaseType_t _result;
	uint32_t _eventFLAGS;
	status_t _status;

	if( NULL == handle )
		return kStatus_InvalidArgument;

	handle->txTransfer.data = (uint8_t*) toSend;
	handle->txTransfer.dataSize = (uint32_t) strlen((char*)toSend);
	handle->SenderHandler = xTaskGetCurrentTaskHandle();
	_status = USART_TransferSendNonBlocking( (USART_Type*)handle->base, &(handle->usart_handle), &(handle->txTransfer));
	if( _status != kStatus_Success )
	{
		return kStatus_Fail;
	}
	_result = xTaskNotifyWait( RTOS_USART_COMPLETE, 0, &_eventFLAGS, timeout );
	if( pdPASS != _result  )
	{
		USART_TransferAbortSend( (USART_Type*)handle->base, &(handle->usart_handle) );
		return kStatus_Timeout;
	}
	if( (RTOS_USART_COMPLETE & _eventFLAGS) != 0 )
				return kStatus_Success;
	return kStatus_Fail;
}

status_t USART_RTOS_Receive( usart_rtos_handle_t *handle, uint8_t* rcvBuff, size_t sizeRcvBuff, TickType_t timeout )
{
	status_t _status;
	usart_rtos_notification_t _taskNotification;

	_taskNotification.transfer.data = rcvBuff;
	_taskNotification.transfer.dataSize = sizeRcvBuff;
	_taskNotification.task = xTaskGetCurrentTaskHandle();

	taskENTER_CRITICAL();
//	handle->rxTransfer.data = (uint8_t*) rcvBuff;
//	handle->rxTransfer.dataSize = (uint32_t) sizeRcvBuff;
	handle->status = USART_RTOS_CMD;
	handle->CMD_handler_notification = &_taskNotification;
//	_status = xTaskNotifyIndexed( handle->ReceiveHandler, USART_RTOS_HANDLER_INDEX, (uint32_t)(&_taskNotification), eSetValueWithOverwrite );
	taskEXIT_CRITICAL();

//	_status = xTaskNotifyWait( 0, 0, (uint32_t*)&rcvBuff, timeout );
	xTaskNotifyStateClear( _taskNotification.task );
	ulTaskNotifyValueClear( _taskNotification.task, UINT32_MAX );
	_status = ulTaskNotifyTake( pdTRUE, timeout );

	if( pdPASS != _status )
	{
//		USART_RTOS_Clean();
		return kStatus_Timeout;
	}


	handle->status = USART_RTOS_IDLE;		//to reset status

	return kStatus_Success;
}

status_t USART_RTOS_Send_Receive( usart_rtos_handle_t *handle, const uint8_t *toSend, uint8_t* rcvBuff, size_t sizeRcvBuff, TickType_t timeout )
{
	status_t _status;
	TimeOut_t _timeout;
//	BaseType_t _result;

	if( NULL == handle )
		return kStatus_InvalidArgument;
	vTaskSetTimeOutState( &_timeout );
	if( xSemaphoreTake( handle->semaphore, timeout ) == pdPASS )
	{
		xTaskCheckForTimeOut( &_timeout, &timeout );

		_status = USART_RTOS_Send( handle, toSend, strlen((char*)toSend), timeout );
		if( kStatus_Fail == _status || kStatus_Timeout == _status )
		{
			(void)xSemaphoreGive(handle->semaphore);
			return _status;
		}

		xTaskCheckForTimeOut( &_timeout, &timeout );
		_status = USART_RTOS_Receive(handle, rcvBuff, sizeRcvBuff, timeout);

		if( kStatus_Fail == _status || kStatus_Timeout == _status )
		{
			//USART_TransferAbortReceive( (USART_Type*)handle->base, &(handle->usart_handle) );
			USART_RTOS_Reset( handle );
//			(void)xSemaphoreGive(handle->semaphore);
			return _status;
		}


		(void)xSemaphoreGive(handle->semaphore);
		return kStatus_Success;
	}
	else
		return kStatus_Fail;
}

void USART_RTOS_giveFromUrc( usart_rtos_handle_t *handle )
{
	taskENTER_CRITICAL();
	handle->status = USART_RTOS_IDLE;
	xTaskNotify( handle->ReceiveHandler, RTOS_USART_URC_FINISHED, eSetBits );
	taskEXIT_CRITICAL();
}


/*==========[Function tasks]=====================================================================================================================*/
static void USART_RTOS_ReceiveTask( void* param )
{
	usart_rtos_handle_t *_handle = (usart_rtos_handle_t *)param;
	size_t _n;
	uint32_t _eventFLAGS;
	status_t _status;
	BaseType_t _result;
	uint8_t _buffer[1] = {0};
	usart_rtos_notification_t *_taskNotification;
	static size_t _bufferIndex = 0;

	//Suspend task to wait USART init finalization
	vTaskSuspend(NULL);

	while(1)
	{
		_handle->rxTransfer.data     = _buffer;
		_handle->rxTransfer.dataSize = sizeof(_buffer);

		if( USART_RTOS_PROCESSING == _handle->status )
		{
			_result = xTaskNotifyWait( RTOS_USART_URC_FINISHED, 0, &_eventFLAGS, RECEIVER_TIMOUT_ERROR );
			if( ( RTOS_USART_URC_FINISHED & _eventFLAGS ) != 0 )
				;	// deberia ser este o OVERRUN
//			continue;
			if( pdFALSE == _result || ( RTOS_USART_RING_BUFFER_OVERRUN & _eventFLAGS ) != 0)
				USART_RTOS_Reset( _handle );

			(void)xSemaphoreGive( _handle->semaphore );
		}

		/***** USART reception *****/
		/* Non-blocking call */
		_status = USART_TransferReceiveNonBlocking( (USART_Type*)_handle->base, &(_handle->usart_handle), &(_handle->rxTransfer), &_n);
		if ( _status != kStatus_Success )
		{
			PRINTF( "ReceiveTask fail. response %i", _status );
			//ver que hacer aca
		}
		_result = xTaskNotifyWait( RTOS_USART_COMPLETE | RTOS_USART_RING_BUFFER_OVERRUN | RTOS_USART_RESET, 0, &_eventFLAGS, RECEIVER_TIMOUT_ERROR );
		/**************************/

		if( pdPASS == _result )
		{
			if( (RTOS_USART_RING_BUFFER_OVERRUN & _eventFLAGS) != 0 )		// Ring Buffer Overrun, reset the comms
			{
				//TODO Log overrun
				PRINTF( "UART%i Overrun\n", wichUsartUsed(_handle) );
//				PRINTF( "Overrun\n" );
				USART_RTOS_Reset( _handle );
				_bufferIndex = 0;
				_taskNotification = NULL;
				_handle->CMD_handler_notification = NULL;
			}
			if( (RTOS_USART_COMPLETE & _eventFLAGS) != 0 )		// a data has arrived
			{
				taskENTER_CRITICAL();
				if( _handle->status == USART_RTOS_IDLE )						// if data arrive and not command send, then must be an URC
				{
					if( xSemaphoreTake( _handle->semaphore, 0 ) == pdPASS )
					{
						_handle->status = USART_RTOS_URC;
						_taskNotification = (usart_rtos_notification_t*)_handle->URC_handler_notification;
					}
					else
					{
						taskEXIT_CRITICAL();
						continue;  // mutex take by sender, but not receiving yet. ignore data //configASSERT(0);		// if enter here then something went wrong.
					}
				}
				else if ( _handle->status == USART_RTOS_CMD )		//if a command was sent
				{
					_taskNotification = (usart_rtos_notification_t*)_handle->CMD_handler_notification;
					if( _taskNotification == NULL )
						USART_RTOS_Reset( _handle );
//					xTaskNotifyWaitIndexed( USART_RTOS_HANDLER_INDEX, 0, 0, (uint32_t*) &_taskNotification, 0 );	//receive which task should notify when reception ends, and where to place it
				}
				taskEXIT_CRITICAL();

				switch (_buffer[0])
				{
				case '\n':
					_taskNotification->transfer.data[_bufferIndex] = '\0';
					if( _bufferIndex > 0 )
					{
						taskENTER_CRITICAL();
//						// if URC give semaphore
//						if( USART_RTOS_URC == _handle->status )
//							(void)xSemaphoreGive(_handle->semaphore);
						//notify to receive
						xTaskNotifyGive( _taskNotification->task );
						if( _handle->status == USART_RTOS_CMD )
							_handle->status = USART_RTOS_IDLE;
						else
							_handle->status = USART_RTOS_PROCESSING;
						_bufferIndex = 0;
						_taskNotification = NULL;
						taskEXIT_CRITICAL();
					}
					break;
				case '\r':
					break;
				case '\b':
					if( _bufferIndex > 0 )
						_bufferIndex--;
					break;
				default:
					_taskNotification->transfer.data[_bufferIndex++] = _buffer[0];
					if( _bufferIndex >= _taskNotification->transfer.dataSize - 1 )
					{
						// response too big to process.
						_taskNotification->transfer.data[_taskNotification->transfer.dataSize - 1] = '\0';
						// send to log and/or debug usart
						//
						PRINTF( "Buffer small\n" );
						PRINTF( "%s", _taskNotification->transfer.data );
						_bufferIndex = 0;
					}
					break;
				}

			}
			if( (RTOS_USART_RESET & _eventFLAGS) != 0 )
			{
				_bufferIndex = 0;
				_taskNotification = NULL;
				_handle->CMD_handler_notification = NULL;
				PRINTF( "Resetting usart\n" );
			}
		}
		else	//timeout event data reception. this never should ocurr except an error on USART connection or the other system.
		{
			if( USART_RTOS_CMD == _handle->status )
			{
				// to protect against infinite wait
				xTaskNotifyGive( _handle->CMD_handler_notification->task );
			}
			USART_RTOS_Reset(_handle);

			PRINTF( "UART%i timed out\n", wichUsartUsed(_handle) );
			//cancelar recepcion
			//limpiar uart?
			//avisar de error
		}
	}
}
