/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    HAL_uart_v1.c
 * @brief   Application entry point.
 */

/*====================[Includes]============================================*/

#include <stdio.h>
#include "board.h"
#include "peripherals.h"			// BOARD_InitBootPeripherals
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S16.h"
#include "fsl_debug_console.h"
#include "fsl_power.h"
#include "fsl_gpio.h"
#include "fsl_pint.h"
#include "fsl_inputmux.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

#include "L86.h"

/*====================[Macros and defines]==================================*/
#define APP_SWITCH_GPIO			BOARD_SW3_GPIO
#define APP_SWITCH_PORT			BOARD_SW3_GPIO_PORT
#define APP_SWITCH_PIN			BOARD_SW3_GPIO_PIN
#define APP_SWITCH_INPUTMUX		kINPUTMUX_GpioPort1Pin9ToPintsel


/*====================[Global variables]====================================*/
uint32_t	switchStatus = 1;

/*====================[Functions declarations]==============================*/
static void testL86_task( void *pvParameters );


/*====================[Callback handlers]===================================*/
static void pint_intr_callback( pint_pin_int_t pintr, uint32_t pmatch_status )
{
	taskENTER_CRITICAL();
	switchStatus = 0;
	taskEXIT_CRITICAL();
}

/*====================[Private Functions]===================================*/

static void switchInit( uint32_t port, uint32_t pin)
{
	IOCON_PinMuxSet(IOCON, port, pin, IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);

	GPIO_PortInit( GPIO, port );
	GPIO_PinInit( GPIO, port, pin, &(gpio_pin_config_t){kGPIO_DigitalInput, 0} );


	/* agregado irq */
	INPUTMUX_Init(INPUTMUX);
	INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, APP_SWITCH_INPUTMUX);
    INPUTMUX_Deinit(INPUTMUX);

    /* Initialize PINT */
    PINT_Init(PINT);

    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableRiseEdge, pint_intr_callback);
    /* Enable callbacks for PINT0 by Index */
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);
}

static uint32_t switchGet( uint32_t port, uint32_t pin )
{
	uint32_t _status;
	taskENTER_CRITICAL();
	_status = switchStatus;
	switchStatus = 1;
	taskEXIT_CRITICAL();
	return _status;
}


/*====================[main Function]======================================*/
/*
 * @brief   Application entry point.
 */
int main(void) {

	/* set BOD VBAT level to 1.65V */
	POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    switchInit( BOARD_SW3_GPIO_PORT, BOARD_SW3_GPIO_PIN );

    PRINTF( "Prueba de L86\n" );

    if ( xTaskCreate(	testL86_task,
    					"Uart_task",
						configMINIMAL_STACK_SIZE + 100,
						NULL,
						uart_task_PRIORITY,
						NULL)
    		!= pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

    vTaskStartScheduler();
    for (;;)
        ;
}


/*====================[Tasks]================================================*/

/*
 * @brief	Tarea que inicializa la comunicación uart con el módulo L86, y luego envía un comando de consulta cuando se pulsa el botón
 */
static void testL86_task(void *pvParameters)
{
    L86_Init( L86UartConfig );

    uint32_t sentence = 0;

    if( pdPASS == L86_getOutput( &sentence ) )
    {
    	PRINTF( "output configured 0x%X", sentence );
    }
    while( pdTRUE )
    {
    	uint32_t _pinValue;
    	_pinValue = switchGet( BOARD_SW3_GPIO_PORT, BOARD_SW3_GPIO_PIN );
		if( !(_pinValue) )
		{
			if( pdPASS == L86_getOutput( &sentence ) )
			{
				PRINTF( "output configured 0x%X\n", sentence );
			}
		}

		vTaskDelay( pdMS_TO_TICKS(5000) );
    }

    L86_setOutput( RMC_OUTPUT | GGA_OUTPUT );

    vTaskSuspend(NULL);
    while(1);
}




