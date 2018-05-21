/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example_freertos
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

#include "ESP8266.h"

uint8_t answer[UART_BUFF_SIZE];
extern SemaphoreHandle_t xUARTSemaphore;
/*#if LEDS_NUMBER <= 2
#error "Board is not equipped with enough amount of LEDs"
#endif*/

#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      1000          /**< Timer period. LED1 timer will expire after 1000 ms */

TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
	

		if(ESP8266_AtSend(answer, "AT+RST", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}
		vTaskDelay(1000);
		
		if(ESP8266_AtSend(answer, "AT", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}
		
		if(ESP8266_AtSend(answer, "AT+GMR", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}	

		if(ESP8266_AtSend(answer, "AT+CWLAP", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}	

		if(ESP8266_AtSend(answer, "AT+CWJAP=\"dd-wrt\",\"bora-bora04\"", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}	
		
		if(ESP8266_AtSend(answer, "AT+CIFSR", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}	
		
		if(ESP8266_AtSend(answer, "AT+PING=\"192.168.1.1\"", "OK")){
			SEGGER_RTT_WriteString(0, (const char*)answer);
			SEGGER_RTT_WriteString(0, (const char*)"\r\n");
		}	
		
    while (true)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);
        vTaskDelay(TASK_DELAY);
    }
}

/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    bsp_board_led_invert(BSP_BOARD_LED_1);
}

int main(void)
{
    ret_code_t err_code;

    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    /* Configure LED-pins as outputs */
    bsp_board_leds_init();
    bsp_board_leds_off();

		Serial_Config(115200UL);
		SEGGER_RTT_WriteString(0, (const char*)"LOG: System started!\r\n");
	
		xUARTSemaphore = xSemaphoreCreateBinary();
    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
   // led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
   // UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }
}

/**
 *@}
 **/
