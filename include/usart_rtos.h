/*
 * uart_rtos.h
 *
 *  Created on: 22 mar. 2021
 *      Author: Santiago
 */

#ifndef USART_RTOS_H_
#define USART_RTOS_H_

/*====================[Includes]============================================*/
#include <stdbool.h>
#include "fsl_usart.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*====================[Macros and defines]==================================*/
/* IOCON pin must be set outside */

#define RTOS_USART_COMPLETE                	( 1U << 0 )
#define RTOS_USART_RING_BUFFER_OVERRUN     	( 1U << 1 )
#define RTOS_USART_HARDWARE_BUFFER_OVERRUN 	( 1U << 2 )
#define RTOS_USART_URC_FINISHED		 		( 1U << 2 )
#define RTOS_USART_RESET					( 1U << 3 )


/*====================[Typedefs enums and structs]==========================*/
/*
 * @brief	enumeration of usart internal status
 */
typedef enum{
	USART_RTOS_IDLE,
	USART_RTOS_CMD,
	USART_RTOS_URC,
	USART_RTOS_PROCESSING,
}USART_RTOS_status;

/*
 * @brief	struct that handle tasks and transfer notifications like
 *
 */
typedef struct{
	TaskHandle_t task;
	usart_transfer_t transfer;
} usart_rtos_notification_t;

/*
 * @brief
 *
 */
typedef struct{
	USART_Type *base;                /*!< USART base address */
	uint32_t baudrate;               /*!< Desired communication speed */
	usart_parity_mode_t parity;      /*!< Parity setting */
	usart_stop_bit_count_t stopbits; /*!< Number of stop bits to use */
	uint8_t *buffer;                 /*!< Buffer for background reception */
	uint32_t buffer_size;            /*!< Size of buffer for background reception */
}usart_rtos_config_t;

/*
 * @brief
 *
 */
typedef struct{
	// USART access
	const USART_Type *base;         /*!< USART base address */
	usart_handle_t usart_handle;	/*!< USART handle structure */

	// transfer
	usart_transfer_t txTransfer;   	/*!< TX transfer structure */
	usart_transfer_t rxTransfer;   	/*!< RX transfer structure */

	// Mutex
	SemaphoreHandle_t semaphore; 	/*!< RX semaphore for resource sharing */

	// Tasks handler
	TaskHandle_t ReceiveHandler;	/*!< Handler of task to notify when line received */
	TaskHandle_t SenderHandler;		/*!< Handler of task to notify when send finished */

	// task notification when done
    volatile usart_rtos_notification_t* URC_handler_notification;	/* URC to task to notify */
    volatile usart_rtos_notification_t* CMD_handler_notification;	/* URC to task to notify */

    // internal usart status
    volatile USART_RTOS_status status;
} usart_rtos_handle_t;


/*====================[Functions declarations]===============================*/
/*
 * @brief
 *
 */
int USART_RTOS_Init( usart_rtos_handle_t* handle, const usart_rtos_config_t *config);

/*
 * @brief
 *
 */
void USART_RTOS_Deinit(usart_rtos_handle_t *handle);

/*
 * @brief
 *
 */
void USART_RTOS_Reset(usart_rtos_handle_t *_handle);

/*
 * @brief
 *
 */
status_t USART_RTOS_Send( usart_rtos_handle_t *handle, const uint8_t* toSend, size_t sizeToSend, TickType_t timeout );

/*
 * @brief
 *
 */
status_t USART_RTOS_Receive( usart_rtos_handle_t *handle, uint8_t* rcvBuff, size_t sizeRcvBuff, TickType_t timeout );

/*
 * @brief
 *
 */
status_t USART_RTOS_Send_Receive( usart_rtos_handle_t *handle, const uint8_t *toSend, uint8_t* rcvBuff, size_t sizeRcvBuff, TickType_t timeout );

/*
 * @brief
 *
 */
status_t USART_RTOS_SetURC(  usart_rtos_handle_t *handle, usart_rtos_notification_t *taskNotification );

/*
 * @brief
 *
 */
void USART_RTOS_giveFromUrc( usart_rtos_handle_t *handle );

#endif /* USART_RTOS_H_ */
