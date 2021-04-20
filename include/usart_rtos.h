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
 * @brief	Configuration of USART
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
 * @brief	handler of used USART
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
 * @brief	Initialize USART according to config, create task to receive bytes, and initialize Ring buffer
 * @param	handle:	pointer to handler of used USART
 * @config	pointer to configuration to use when initialize USART
 */
status_t USART_RTOS_Init( usart_rtos_handle_t* handle, const usart_rtos_config_t *config);

/*
 * @brief	Deinitialize USART and delete task
 * @param	handle:	pointer to handler of used USART
 *
 */
void USART_RTOS_Deinit(usart_rtos_handle_t *handle);

/*
 * @brief	Clean Tx, Rx buffers, and give mutex if taken
 * @param	handle:	pointer to handler of used USART
 *
 */
void USART_RTOS_Reset(usart_rtos_handle_t *_handle);

/*
 * @brief	Starts a Tx transfer or timeout if coudnt send it
 * @param	handle:	pointer to handler of used USART
 * @param	toSend: pointer to string to send through handle. Must be a '\0' ended string.
 * @param	timeout: ticks to wait before exit and cancel transaction
 */
status_t USART_RTOS_Send( usart_rtos_handle_t *handle, const uint8_t* toSend, TickType_t timeout );

/*
 * @brief	Starts an Rx transfer or timeout if no string received
 * @param	handle:	pointer to handler of used USART
 * @param	rcvBuff: pointer to buffer where received string will be stored
 * @param	sizeRcvBuff: size of buffer rcvBuff
 * @param	timeout: ticks to wait before exit and cancel transaction
 *
 */
status_t USART_RTOS_Receive( usart_rtos_handle_t *handle, uint8_t* rcvBuff, size_t sizeRcvBuff, TickType_t timeout );

/*
 * @brief	Starts a Tx, and after that an Rx transfer. if coudnt send or receive the string, timeout
 * @param	handle:	pointer to handler of used USART
 * @param	toSend: pointer to string to send through handle. Must be a '\0' ended string.
 * @param	rcvBuff: pointer to buffer where received string will be stored
 * @param	sizeRcvBuff: size of buffer rcvBuff
 * @param	timeout: ticks to wait before exit and cancel transaction
 *
 */
status_t USART_RTOS_Send_Receive( usart_rtos_handle_t *handle, const uint8_t *toSend, uint8_t* rcvBuff, size_t sizeRcvBuff, TickType_t timeout );

/*
 * @brief	Sets task to notify when there are no transfer in progress but data ahs arrived
 * @param	handle:	pointer to handler of used USART
 * @param	taskNotification: pointer to struct that holds the task and buffer that will be used when URC string comes
 */
status_t USART_RTOS_SetURC(  usart_rtos_handle_t *handle, usart_rtos_notification_t *taskNotification );

/*
 * @brief	Used to notify end of URC process to receiver task
 * @param	handle:	pointer to handler of used USART
 *
 */
void USART_RTOS_giveFromUrc( usart_rtos_handle_t *handle );

#endif /* USART_RTOS_H_ */
