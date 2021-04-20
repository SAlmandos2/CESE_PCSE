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
#define APP_SWITCH0_GPIO		BOARD_SW3_GPIO
#define APP_SWITCH0_PORT		BOARD_SW3_GPIO_PORT
#define APP_SWITCH0_PIN			BOARD_SW3_GPIO_PIN
#define APP_SWITCH0_INT_INDEX	kPINT_PinInt0
#define APP_SWITCH0_INPUTMUX	kINPUTMUX_GpioPort1Pin9ToPintsel

#define APP_SWITCH1_GPIO		BOARD_SW1_GPIO
#define APP_SWITCH1_PORT		BOARD_SW1_GPIO_PORT
#define APP_SWITCH1_PIN			BOARD_SW1_GPIO_PIN
#define APP_SWITCH1_INT_INDEX	kPINT_PinInt1
#define APP_SWITCH1_INPUTMUX	kINPUTMUX_GpioPort1Pin18ToPintsel

/*====================[Global variables]====================================*/
uint32_t	switch3Status = 1;
uint32_t	switch1Status = 1;

// Used to print output configuration
const char *urc[] = {	"GLL",
						"RMC",
						"VTG",
						"GGA",
						"GSA",
						"GSV",
						"GRS",
						"GST",
						"ZDA",
						"MCHN"
					};

// Used to print output configuration
const uint32_t urcFilter[] = { 	GLL_OUTPUT,
								 RMC_OUTPUT,
								 VTG_OUTPUT,
								 GGA_OUTPUT,
								 GSA_OUTPUT,
								 GSV_OUTPUT,
								 GRS_OUTPUT,
								 GST_OUTPUT,
								 ZDA_OUTPUT,
								 MCHN_OUTPUT
							};
/*====================[Functions declarations]==============================*/
static void testL86_task( void *pvParameters );


/*====================[Callback handlers]===================================*/
static void pint_intr_callback( pint_pin_int_t pintr, uint32_t pmatch_status )
{
	if( APP_SWITCH0_INT_INDEX == pintr )
	{
		taskENTER_CRITICAL();
		switch3Status = 0;
		taskEXIT_CRITICAL();
	}
	else if( APP_SWITCH1_INT_INDEX == pintr )
	{
		taskENTER_CRITICAL();
		switch1Status = 0;
		taskEXIT_CRITICAL();
	}
}

/*====================[Private Functions]===================================*/

static void switchInit( )
{
	GPIO_PortInit( GPIO, APP_SWITCH0_PORT );
	GPIO_PortInit( GPIO, APP_SWITCH1_PORT );

	IOCON_PinMuxSet(IOCON, APP_SWITCH0_PORT, APP_SWITCH0_PIN, IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);
	IOCON_PinMuxSet(IOCON, APP_SWITCH1_PORT, APP_SWITCH1_PIN, IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN);

	GPIO_PinInit( APP_SWITCH0_GPIO, APP_SWITCH0_PORT, APP_SWITCH0_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0} );
	GPIO_PinInit( APP_SWITCH1_GPIO, APP_SWITCH1_PORT, APP_SWITCH1_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0} );

	/* agregado irq */
	INPUTMUX_Init(INPUTMUX);
	INPUTMUX_AttachSignal(INPUTMUX, APP_SWITCH0_INT_INDEX, APP_SWITCH0_INPUTMUX);
	INPUTMUX_AttachSignal(INPUTMUX, APP_SWITCH1_INT_INDEX, APP_SWITCH1_INPUTMUX);
    INPUTMUX_Deinit(INPUTMUX);

    /* Initialize PINT */
    PINT_Init(PINT);

    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, APP_SWITCH0_INT_INDEX, kPINT_PinIntEnableRiseEdge, pint_intr_callback);
    /* Enable callbacks for PINT0 by Index */
    PINT_EnableCallbackByIndex(PINT, APP_SWITCH0_INT_INDEX);

    /* Setup Pin Interrupt 1 for rising edge */
    PINT_PinInterruptConfig(PINT, APP_SWITCH1_INT_INDEX, kPINT_PinIntEnableRiseEdge, pint_intr_callback);
    /* Enable callbacks for PINT1 by Index */
    PINT_EnableCallbackByIndex(PINT, APP_SWITCH1_INT_INDEX);
}

static uint32_t switchGet( uint32_t port, uint32_t pin )
{
	uint32_t _status;

	if( port == APP_SWITCH0_PORT && pin == APP_SWITCH0_PIN )
	{
		taskENTER_CRITICAL();
		_status = switch3Status;
		switch3Status = 1;
		taskEXIT_CRITICAL();
	}
	else if(  port == APP_SWITCH1_PORT && pin == APP_SWITCH1_PIN )
	{
		taskENTER_CRITICAL();
		_status = switch1Status;
		switch1Status = 1;
		taskEXIT_CRITICAL();
	}
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
    switchInit();

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

    uint32_t sentence = RMC_OUTPUT | GGA_OUTPUT;

    while( pdTRUE )
    {
    	uint32_t _sw1Value;
    	_sw1Value = switchGet( APP_SWITCH1_PORT, APP_SWITCH1_PIN );
		if( !(_sw1Value) )
		{
			if( pdPASS == L86_getOutput( &sentence ) )
			{
				int i;
				char str[64] = {0};
				strcpy( str, "nmea ouptput enabled:" );
				for( i = 0 ; i <= ( sizeof( urcFilter ) / sizeof( urcFilter[0] ) ) ; i++)
				{
					bool enabled = IS_NMEA_ENABLED( sentence & NMEA_OUTPUT_MASK, urcFilter[i] );
					if( enabled )
					{
						strcat(str, " " );
					}
					strcat(str, enabled ? urc[i] : "" );
				}
				strcat(str, "\n");
				PRINTF( "%s", str);
			}
		}

		uint32_t _sw3Value;
		_sw3Value = switchGet( APP_SWITCH0_PORT, APP_SWITCH0_PIN );
		if( !(_sw3Value) )
		{
			sentence ^= VTG_OUTPUT;
			if( pdPASS == L86_setOutput( sentence ) )
			{
				PRINTF( "Trama VTG %s\n", IS_NMEA_ENABLED( sentence, VTG_OUTPUT ) ? "Activada" : "Desactivada" );
			}
		}

		vTaskDelay( pdMS_TO_TICKS(5000) );
    }

    L86_setOutput( RMC_OUTPUT | GGA_OUTPUT );

    vTaskSuspend(NULL);
    while(1);
}




