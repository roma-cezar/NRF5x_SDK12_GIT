/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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

#include "nrf_esb.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"

#include "Serial.h"
#include "FLASH.h"
#include "radio.h"

#if defined (NRF51822)
		#include "nrf51.h"
		#include "nrf51_bitfields.h"
#elif defined (NRF52832)
		#include "nrf52.h"
		#include "nrf52_bitfields.h"
#endif
#if defined (NRF51822)
	#include "nrf51_adc.h"
#elif defined (NRF52832)
	#include "nrf52_adc.h"
#endif

#define LED_0 	21
/*************************************************************/
uint32_t timer_1=0;
uint8_t text_buf[128];

extern nrf_esb_payload_t 			rx_payload;
extern nrf_esb_payload_t      tx_payload;
extern device_load_t 					device_load;

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
/*************************************************************/
void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}
/*************************************************************/
void gpio_init( void )
{
    //bsp_board_leds_init();// Светодиод
		// Enable Reset pin
		NRF_POWER->RESET = 1UL;
		nrf_gpio_cfg_output(LED_0); 
}
/*************************************************************/

/*************************************************************/
void TIMER0_IRQHandler(void)
{
	if(NRF_TIMER0->EVENTS_COMPARE[0] == 1)
	{
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->TASKS_STOP = 1;
		NRF_TIMER0->TASKS_CLEAR = 1;
		timer_1++;
		if(timer_1==70)
		{
			nrf_gpio_pin_toggle(LED_0);		
			timer_1 = 0;
			
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
		NRF_TIMER0->TASKS_START = 1;
	}
}

/*************************************************************/
void Timer_Init(void)
{		
		NVIC_DisableIRQ(TIMER0_IRQn);
		NRF_TIMER0->TASKS_STOP = 1;
		NRF_TIMER0->TASKS_CLEAR = 1;
		//Ft = 16000000 / (2^PRESCALER)
		NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
		NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
		NRF_TIMER0->PRESCALER = 10; // 244.8 Hz
		NRF_TIMER0->CC[0] = 16; // 1 mil sec interrupt
		NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
		NVIC_SetPriority(TIMER0_IRQn, 3);
		NVIC_EnableIRQ(TIMER0_IRQn);
		NRF_TIMER0->TASKS_START = 1;
}

/*************************************************************/
int main(void)
{
    uint32_t err_code;

    gpio_init();
    clocks_start();
		Serial_Config(115200);
	
		while(!CheckCmd("start"));
		FlushRx();
	
		current_channel  = radio_channel_1;
		radio_config(RX);
		radio_set_channel(current_channel);
		radio_listen(START);
	
		Timer_Init();
		#ifdef DEBUG
			Serial_Print((uint8_t*)"DEBUG: Gateway started!\r\n");
		#endif
	
    while (true)
    {	
			if(CheckCmd("reboot"))
			{
				FlushRx();
				#ifdef DEBUG
					Serial_Print((uint8_t*)"DEBUG: Rebooting...\r\n");
				#endif
				nrf_delay_ms(100);
				NVIC_SystemReset();
			}

			
			
			if(tx_done_flag)
			{
				__enable_irq();
				tx_done_flag = false;
				
				if(device_bond_flag == true)
				{
					device_bond_flag = false;
					if(GatewayParseCmd(&device_load))
					{
						nrf_gpio_pin_set(LED_0);
						nrf_delay_ms(250);						
						nrf_gpio_pin_clear(LED_0);	
						nrf_delay_ms(100);	
						nrf_gpio_pin_set(LED_0);
						nrf_delay_ms(250);						
						nrf_gpio_pin_clear(LED_0);	
						nrf_delay_ms(100);			
						nrf_gpio_pin_set(LED_0);	
						nrf_delay_ms(250);			
						nrf_gpio_pin_clear(LED_0);	
						nrf_delay_ms(100);			
					}
				}

				radio_config(RX);
				radio_listen(START);
				NRF_TIMER0->TASKS_START = 1;
			}
			if(tx_fail_flag)
			{
				__enable_irq();
				tx_fail_flag = false;
				radio_config(RX);
				radio_listen(START);
				NRF_TIMER0->TASKS_START = 1;
			}
			if(rx_done_flag)
			{
				__enable_irq();
				rx_done_flag = false;
				NRF_TIMER0->TASKS_STOP = 1;
				radio_listen(STOP);
				nrf_gpio_pin_clear(LED_0);	
				/***********************************************************************************************/
				/***********************************************************************************************/
				
				/***********************************************************************************************/
				/***********************************************************************************************/
				if(rx_payload.pipe == PIPE_TYPE_GERKON) // Если пришло сообщение на адрес геркона
				{				
					if(rx_payload.data[0] == MASSAGE_TYPE_CMD && rx_payload.data[6] == CMD_BOND )
					{
							device_bond_flag = true;
						
							#ifdef DEBUG
//								memset(text_buf, 0, sizeof(text_buf));
//								sprintf(text_buf, "DEBUG: Device pairing request on channel %d\r\n",  current_channel);
//								Serial_Print((uint8_t*)text_buf);
							#endif
						
							radio_config(TX);
							radio_send_bond_ok(&device_load);
					}
					
					else if(rx_payload.data[0] == MASSAGE_TYPE_TELE)
					{
						#ifdef DEBUG
//							memset(text_buf, 0, sizeof(text_buf));
//							sprintf(text_buf, "DEBUG: Device incoming data on channel %d\r\n",  current_channel);
//							Serial_Print((uint8_t*)text_buf);
						#endif
						if(GatewayParseData(&device_load))
						{
							#ifdef DEBUG
//								memset(text_buf, 0, sizeof(text_buf));
//								sprintf(text_buf, "DEBUG: Send OK to 0x%.8X\r\n",  device_load.device_id);
//								Serial_Print((uint8_t*)text_buf);
							#endif
							radio_config(TX);
							radio_send_ok(&device_load);
						}
						else
						{
							radio_config(RX);				
							radio_listen(START);
							NRF_TIMER0->TASKS_START = 1;
						}
					}
					else
					{				
						radio_listen(START);
						NRF_TIMER0->TASKS_START = 1;
					}
				}
				else
				{				
					radio_listen(START);
					NRF_TIMER0->TASKS_START = 1;
				}
			}
		}
}


/*lint -restore */
