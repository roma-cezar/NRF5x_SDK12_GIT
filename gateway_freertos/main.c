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
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"


#include "nrf_esb.h"
#include "Serial.h"
#include "radio.h"

#define LED_0 	21

#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      200          /**< Timer period. LED1 timer will expire after 1000 ms */

TaskHandle_t  task1_handle;   
TaskHandle_t  task2_handle;  
TaskHandle_t  task3_handle; 
TimerHandle_t freq_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */

extern QueueHandle_t xQueueRF;
extern QueueHandle_t xQueueUART;

extern SemaphoreHandle_t xUARTSemaphore;

extern RadioMessage myMessageToSend;
extern RadioMessage myMessageToRecieve;
extern device_load_t device_load;

char str_buff[ 256 ];

extern bool device_send_flag;
extern bool device_bond_flag;

extern bool rx_done_flag;
extern bool tx_done_flag;
extern bool tx_fail_flag;

uint8_t channel_sw_cnt = 0;
extern uint8_t current_channel;
extern uint8_t radio_channel_1;
extern uint8_t radio_channel_2;
extern uint8_t radio_channel_3;

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */

static void task1_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
		uint8_t textbuff[128];
    while (true)
    {
			
		if(tx_done_flag)
		{
			tx_done_flag = false;
			
			if(device_bond_flag == true)
			{
				device_bond_flag = false;
				if(GatewayParseCmd(&device_load, &myMessageToRecieve))
				{
					nrf_gpio_pin_set(LED_0);
					vTaskDelay(250);						
					nrf_gpio_pin_clear(LED_0);	
					vTaskDelay(100);	
					nrf_gpio_pin_set(LED_0);
					vTaskDelay(250);						
					nrf_gpio_pin_clear(LED_0);	
					vTaskDelay(100);			
					nrf_gpio_pin_set(LED_0);	
					vTaskDelay(250);			
					nrf_gpio_pin_clear(LED_0);	
					vTaskDelay(100);			
				}
			}

			radio_config(RX);
			radio_listen(START);
			xTimerStart(freq_timer_handle, 0);
		}	
		if(tx_fail_flag)
		{
			tx_fail_flag = false;
			radio_config(RX);
			radio_listen(START);
			xTimerStart(freq_timer_handle, 0);
		}
		
		if(rx_done_flag)
		{	
			if( xQueueRF != 0 && uxQueueMessagesWaiting(xQueueRF) > 0)
			{
					
				rx_done_flag = false;
				radio_listen(STOP);
				xTimerStop(freq_timer_handle, 0);	
				while(uxQueueMessagesWaiting(xQueueRF) != 0)
				{
					if( xQueueReceive( xQueueRF, (void *)&myMessageToRecieve, ( TickType_t ) 0 ) != pdPASS )
					{
						#ifdef DEBUG
						Serial_Print("DEBUG: Queue message receive failed!\r\n");
						#endif
					}
					else
					{
						#ifdef DEBUG
						Serial_Print("DEBUG: Queue message received\r\n");
						#endif						
						nrf_gpio_pin_toggle(LED_0);
						if(GatewayParseData(&device_load, &myMessageToRecieve))
						{
							#ifdef DEBUG
							Serial_Print("DEBUG: Send ok to device!\r\n");
							#endif
							radio_config(TX);
							radio_send_ok(&device_load);
						}
						else
						{				
							radio_listen(START);
							xTimerStart(freq_timer_handle, 0);
						}
					}
				}
				
			}
		}
		
		//vTaskDelay(5);
	}
}

static void task2_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {					
        bsp_board_led_invert(BSP_BOARD_LED_0);
			
				NRF_UART0->TASKS_STARTRX = 1;
				if( xSemaphoreTake( xUARTSemaphore, portMAX_DELAY ))
				{
					if(CheckCmd("reboot"))
					{
						FlushRx();
						Serial_Print((uint8_t*)"Rebooting...\r\n");
						vTaskDelay(500);
						NVIC_SystemReset();
					}
				}
        vTaskDelay(5);
    }
}


static void task3_function (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	while (true)
	{				
		bsp_board_led_invert(BSP_BOARD_LED_1);
		vTaskDelay(100);
	}
	
}
/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void freq_timer_callback (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	nrf_gpio_pin_toggle(LED_0);

	channel_sw_cnt++;
	if(channel_sw_cnt == 1)
	{
		current_channel = radio_channel_1;
		radio_change_rx_channel(current_channel);
	}
	else if(channel_sw_cnt == 2) 
	{
		current_channel = radio_channel_2;
		radio_change_rx_channel(current_channel);
	}
	else if(channel_sw_cnt >= 3) 
	{
		current_channel = radio_channel_3;
		radio_change_rx_channel(current_channel);
		channel_sw_cnt = 0;
	}				
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

int main(void)
{
    ret_code_t err_code;

		clocks_start();

    bsp_board_leds_init();
    bsp_board_leds_off();

		Serial_Config(115200);	
	
		if(radio_config(RX))
		{
			current_channel = radio_channel_1;
			if(radio_set_channel(current_channel)){
				#ifdef DEBUG
					Serial_Print((uint8_t*)"DEBUG: Gateway running...\r\n");
				#endif
				radio_listen(START);
			}
		}
	
	  #ifdef DEBUG
			Serial_Print("DEBUG: Create Queue...\r\n");
		#endif
		
    xUARTSemaphore = xSemaphoreCreateBinary();
		xQueueRF  = xQueueCreate( 10, sizeof( RadioMessage ) );	
		
		#ifdef DEBUG
			Serial_Print("DEBUG: Create tasks...\r\n");
		#endif
		
    xTaskCreate(task1_function, "RADIO", configMINIMAL_STACK_SIZE + 120, NULL, 1, &task1_handle);
    xTaskCreate(task2_function, "UART",  configMINIMAL_STACK_SIZE + 60 , NULL, 2, &task2_handle);
   // xTaskCreate(task3_function, "LED1",  configMINIMAL_STACK_SIZE + 60 , NULL, 3, &task3_handle);

    /* Start timer for LED1 blinking */
    freq_timer_handle = xTimerCreate( "FREQ", TIMER_PERIOD, pdTRUE, NULL, freq_timer_callback);
    xTimerStart(freq_timer_handle, 0);

		#ifdef DEBUG
			Serial_Print("DEBUG: Start FreeRTOS scheduler...\r\n");
		#endif
		
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
