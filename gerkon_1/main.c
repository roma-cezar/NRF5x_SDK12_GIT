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
		#include "nrf51_adc.h"
		#define LED_0 	18
		#define LED_1 	19
		#define GERKON 	29
		#define BUTTON 	28
#elif defined (NRF52832)
		#include "nrf52.h"
		#include "nrf52_bitfields.h"
		#include "nrf52_adc.h"
		extern bool SAADC_DATAREADY_FLAG;
		#define LED_0 	9
		#define GERKON 	8
		#define BUTTON 	10
#endif

#define XTAL_HF 		0
#define XTAL_LF 		1

#define RAM_RETENTION_OFF       (0x00000003UL)

/*****************************************************************************************************/
uint32_t timer_1 = 0;
uint8_t gerkon_state[1];

extern bool device_send_flag;

bool device_send_bond_flag = false;
extern bool device_bond_flag;
extern bool device_bond_enable;

extern bool rx_done_flag;
extern bool tx_done_flag;
extern bool tx_fail_flag;

extern nrf_esb_payload_t 			rx_payload;
extern nrf_esb_payload_t      tx_payload;
extern device_load_t 				device_load;

extern uint8_t current_channel;
extern uint8_t radio_channel_1;
extern uint8_t radio_channel_2;
extern uint8_t radio_channel_3;

uint8_t retry_cnt = 0;
uint8_t retry_tim_cnt = 0;
uint8_t ok_retry_cnt = 0;

uint8_t device_lost_packets = 0;
uint8_t channel_sw_cnt = 0;
uint32_t saved_gateway_id = 0;

uint16_t VBAT = 0;		
bool lpm_flag = true;

uint8_t rnd_delay = 0;
/*****************************************************************************************************/
uint8_t get_rng_val(void)
{
	uint8_t val = 0;
	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled << RNG_CONFIG_DERCEN_Pos;
	NRF_RNG->TASKS_START = 1;
	while (NRF_RNG->EVENTS_VALRDY == 0);
	NRF_RNG->TASKS_STOP = 1;
	val = ((uint8_t)NRF_RNG->VALUE) / 10;
	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Disabled << RNG_CONFIG_DERCEN_Pos;
	if (val < 2) val = 2;
	return val;
}

bool clocks_set(uint8_t source, bool state )
{
	NRF_CLOCK->CTIV = 16;
	if(source == XTAL_HF)
	{
		if(state == true)
		{
				#if defined (NRF51822)
						NRF_CLOCK->XTALFREQ = CLOCK_XTALFREQ_XTALFREQ_16MHz << CLOCK_XTALFREQ_XTALFREQ_Pos;
				#endif
				NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
				NRF_CLOCK->TASKS_HFCLKSTART = 1;
				while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
		}
		else if(state == false){
			// если генератор запущен
			NRF_CLOCK->TASKS_HFCLKSTOP = 1;
			__NOP;
		}
		return (bool)(NRF_CLOCK->HFCLKSTAT >> CLOCK_HFCLKSTAT_STATE_Pos);
	}
	else if(source == XTAL_LF)
	{
		if(state == true)
		{
			// если генератор не запущен
			NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
			NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
			NRF_CLOCK->TASKS_LFCLKSTART = 1;
			while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
		}
		else if(state == false){
			NRF_CLOCK->TASKS_LFCLKSTOP = 1;
			__NOP;
		}
		return (bool)(NRF_CLOCK->LFCLKSTAT >> CLOCK_LFCLKSTAT_STATE_Pos);
	}
}
/*****************************************************************************************************/
void gpio_init( void )
{
	uint32_t err_code;
	//bsp_board_leds_init();
	NVIC_DisableIRQ(GPIOTE_IRQn);	
	NRF_GPIOTE->EVENTS_PORT = 0;
    #if defined (NRF51822)
	#elif defined (NRF52832)
		// Disable NFC pins
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    NRF_UICR->NFCPINS = UICR_NFCPINS_PROTECT_Disabled << UICR_CUSTOMER_CUSTOMER_Pos;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren<< NVMC_CONFIG_WEN_Pos;
	#endif
	// Светодиод - LED
	nrf_gpio_cfg_output(LED_0); 
	// Кнопка - Button
	nrf_gpio_cfg_sense_input(BUTTON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos, NRF_GPIO_PIN_SENSE_LOW); 
	// Геркон - Gerkon
	nrf_gpio_cfg_sense_input(GERKON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos, NRF_GPIO_PIN_SENSE_LOW); 
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;   
}
/*****************************************************************************************************/
void GPIOTE_IRQHandler(void)
{	
	lpm_flag = true;
	if(NRF_GPIOTE->EVENTS_PORT)
	{
		lpm_flag = false; //не используем спящий режим во время общения
		clocks_set(XTAL_LF, false );
		clocks_set(XTAL_HF, true );
		NVIC_DisableIRQ(GPIOTE_IRQn);
		NRF_GPIOTE->EVENTS_PORT = 0;
		if( ((~NRF_GPIO->IN) >> BUTTON) & 0x01)
		{		
				device_bond_enable = true;		
		}	
		else if( ((~NRF_GPIO->IN) >> GERKON) & 0x01 )
		{			
				nrf_gpio_cfg_sense_input(GERKON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos,
																 NRF_GPIO_PIN_SENSE_HIGH); 	// Interrupt low-to-high for close
				gerkon_state[0] = 1;
				device_send_flag = true;
		}
		else if( !(((~NRF_GPIO->IN) >> GERKON) & 0x01) )
		{		
				nrf_gpio_cfg_sense_input(GERKON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos, 
																 NRF_GPIO_PIN_SENSE_LOW); 	// Interrupt high-to-low for close
				gerkon_state[0] = 0;
				device_send_flag = true;
		}
	}
}
/*****************************************************************************************************/
void TIMER0_IRQHandler(void)
{
	// здесь ожидаем таймаут ожидания ответа от станции
	if(NRF_TIMER0->EVENTS_COMPARE[0] == 1)
	{
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->TASKS_CLEAR = 1; // очистим таймер
		NRF_TIMER0->TASKS_STOP  = 1; // остановим таймер
		device_lost_packets++;
		
		radio_listen(STOP); // останавливаем прием							
		radio_config(TX); // переводим в режим передатчика
		if(retry_tim_cnt++ < 6)
		{
			//lpm_flag = false;
					
			channel_sw_cnt++;
			if(channel_sw_cnt == 1)
			{
				current_channel = radio_channel_1;
			}
			else if(channel_sw_cnt == 2) 
			{
				current_channel = radio_channel_2;
			}
			else if(channel_sw_cnt >= 3) 
			{
				current_channel = radio_channel_3;
				channel_sw_cnt = 0;
			}	
			radio_set_channel(current_channel);
			device_load.device_freq_n 			= current_channel;
			device_load.device_lost_packs 	= device_lost_packets;
			NRF_TIMER0->TASKS_START  = 1; // остановим таймер
			radio_send_payload(&device_load);
		}
		else
		{	
			NRF_TIMER0->TASKS_STOP  = 1; // остановим таймер
			NRF_TIMER0->TASKS_SHUTDOWN = 1; // и выключаем таймер
			retry_tim_cnt = 0;
			lpm_flag = true;
		}
		
	}
}
/*****************************************************************************************************/
void Timer_Init(void)
{		
	//Ft = 16000000 / (2^PRESCALER)
	NVIC_DisableIRQ(TIMER0_IRQn);
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->TASKS_STOP = 1;
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER0->PRESCALER = 4; // 244.8 Hz
	NRF_TIMER0->CC[0] = 2000000; // 0.1 sec interrupt
	NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
	NVIC_SetPriority(TIMER0_IRQn, 3);
	NVIC_EnableIRQ(TIMER0_IRQn);
}

/*************************************************************/
int main(void)
{
    uint32_t err_code;
    clocks_set(XTAL_HF, true );
    clocks_set(XTAL_LF, false );

    flash_read(FLASH_USER_START_ADDR, &saved_gateway_id, 1);

		//NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled << POWER_DCDCEN_DCDCEN_Pos;
    gpio_init();
		nrf_gpio_pin_set(LED_0);
		nrf_delay_ms(1000); 
		nrf_gpio_pin_clear(LED_0);	
		nrf_delay_ms(1000); 
	
//    Serial_Config(115200);
    #if defined (NRF51822)
        nrf_adc_init_vbat();
        VBAT = nrf_adc_read_vbat_mv();
    #elif defined (NRF52832)
        SAADC_Config();
        SAADC_StartMeasure();
				if(SAADC_DATAREADY_FLAG==true)
				{
					SAADC_DATAREADY_FLAG=false;
				}
        SAADC_Disable();
    #endif

    current_channel = radio_channel_1;
    lpm_flag = true;

    Timer_Init(); // Настироим таймер таймаута приема
    while (true)
    {	
			/***********************************************************************************************/
			/*****************************************BOND DEVICE*******************************************/
			/***********************************************************************************************/
			if(device_bond_enable)
			{
				lpm_flag = false;
				device_bond_enable = false;
				nrf_gpio_pin_set(LED_0);
				nrf_delay_ms(2000); 
				nrf_gpio_pin_clear(LED_0);	
				
				if(((~NRF_GPIO->IN) >> BUTTON) & 0x01) // если все еще нажата то
				{
					nrf_gpio_pin_set(LED_0);		
					device_bond_flag=true;
					// Заполняем поля данных
					device_load.device_msg_type 		= MASSAGE_TYPE_TELE;
					device_load.device_type 				= DEVICE_TYPE_GERKON;
					device_load.device_id 					= DEVICE_ID;
					// Отправдяем команду на сопряжение
					radio_config(TX);
					radio_set_channel(current_channel);
					radio_send_bond(&device_load);
				}
				else{
					// если нажата менее 3-4 секунд
					lpm_flag = true;		
					device_bond_flag=false;
					nrf_delay_ms(200); // Ждем 0.2 секунды
					nrf_gpio_pin_set(LED_0);
					nrf_delay_ms(100); 
					nrf_gpio_pin_clear(LED_0);		
					nrf_delay_ms(200); // Ждем 0.2 секунды
					nrf_gpio_pin_set(LED_0);
					nrf_delay_ms(100); 
					nrf_gpio_pin_clear(LED_0);	
				}
			}
			
			/***********************************************************************************************/
			/**************************************DEVICE BOND EVENTS***************************************/
			/***********************************************************************************************/
			if(device_bond_flag)
			{
				if(tx_fail_flag)
				{
					tx_fail_flag = false;
					nrf_esb_flush_tx();
					rnd_delay = get_rng_val();
					if(retry_cnt++ < 100)
					{
						nrf_delay_ms(rnd_delay);
						radio_config(TX);
						radio_set_channel(current_channel);
						radio_send_bond(&device_load);
					}
					else{
						retry_cnt = 0;
						device_bond_flag = false;
						lpm_flag = true;
					}
				}
				if(tx_done_flag)
				{
					tx_done_flag = false;
					retry_cnt = 0;
					radio_config(RX);
					radio_listen(START);
				}
				if(rx_done_flag)
				{
					rx_done_flag = false;
					radio_listen(STOP); // остановим приемник
					radio_config(TX); // сконфигурируем передатчик
					
//					if(rx_payload.pipe == PIPE_TYPE_GERKON) // Если пришло сообщение на адрес геркона
//					{
					uint32_t rx_id = 	(((uint32_t)rx_payload.data[2])<<24) |
										(((uint32_t)rx_payload.data[3])<<16) |
										(((uint32_t)rx_payload.data[4])<<8)  |
										(((uint32_t)rx_payload.data[5]));
				
					uint32_t gateway_id = (((uint32_t)rx_payload.data[6])<<24) |
													 (((uint32_t)rx_payload.data[7])<<16) |
													 (((uint32_t)rx_payload.data[8])<<8)  |
													 (((uint32_t)rx_payload.data[9]));
				
					/*******************************************/
					/*Если пришли данные подтверждения привязки*/
					/*******************************************/
					if(	rx_payload.data[1] == DEVICE_TYPE_GERKON &&
						rx_id == DEVICE_ID &&
						rx_payload.data[0] == MASSAGE_TYPE_CMD && 
						rx_payload.data[10] == CMD_BOND_OK &&
						device_bond_flag == true) // если получен ответ ОК
					{
						// Получаем и сохраняем в память адрес гэйтвея
						flash_page_erase(FLASH_USER_START_ADDR);
						flash_write(FLASH_USER_START_ADDR, &gateway_id, 1);
						nrf_delay_ms(5);
						nrf_gpio_pin_clear(LED_0); // выключим светодиод
						// перезагрузимся , что бы применить новые параметры
						NVIC_SystemReset();
					}
//					}
					device_bond_flag = false;
					lpm_flag = true;
				}
			}
			/***********************************************************************************************/
			/*************************************TX/RX DATA DEVICE EVENTS**********************************/
			/***********************************************************************************************/
			else
			{
				if(tx_fail_flag)
				{
					tx_fail_flag = false;
					nrf_esb_flush_tx();
					
					device_lost_packets++;
					
					rnd_delay = get_rng_val();
					if(retry_cnt++ < 250)
					{
						nrf_delay_ms(rnd_delay);
						channel_sw_cnt++;
						if(channel_sw_cnt == 1)
						{
							current_channel = radio_channel_1;
						}
						else if(channel_sw_cnt == 2) 
						{
							current_channel = radio_channel_2;
						}
						else if(channel_sw_cnt >= 3) 
						{
							current_channel = radio_channel_3;
							channel_sw_cnt = 0;
						}	
						
						radio_config(TX);
						radio_set_channel(current_channel);
						device_load.device_freq_n 			= current_channel;
						device_load.device_lost_packs 	= device_lost_packets;
						radio_send_payload(&device_load);
					}
					else{
						retry_cnt = 0;
						lpm_flag = true;
					}
				}
				if(tx_done_flag)
				{
					tx_done_flag = false;
					retry_cnt = 0;
					radio_config(RX);
					radio_listen(START);
					NRF_TIMER0->TASKS_START = 1;
				}
				if(rx_done_flag)
				{
					rx_done_flag = false;
					NRF_TIMER0->TASKS_STOP  = 1; // Останавливаем таймер
					NRF_TIMER0->TASKS_SHUTDOWN = 1; // и выключаем таймер
					radio_listen(STOP); // остановим приемник
					radio_config(TX); // сконфигурируем передатчик
					
					if(rx_payload.pipe == PIPE_TYPE_GERKON) // Если пришло сообщение на адрес геркона
					{
						uint32_t rx_id = 	(((uint32_t)rx_payload.data[2])<<24) |
											(((uint32_t)rx_payload.data[3])<<16) |
											(((uint32_t)rx_payload.data[4])<<8)  |
											(((uint32_t)rx_payload.data[5]));
					
						uint32_t gateway_id = (((uint32_t)rx_payload.data[6])<<24) |
														 (((uint32_t)rx_payload.data[7])<<16) |
														 (((uint32_t)rx_payload.data[8])<<8)  |
														 (((uint32_t)rx_payload.data[9]));
					
						/*******************************************/
						/**Если пришло подтверждение приема данных**/
						/*******************************************/
						if(	rx_payload.data[1] == DEVICE_TYPE_GERKON &&
							gateway_id == saved_gateway_id && //
							rx_id == DEVICE_ID &&
							rx_payload.data[0] == MASSAGE_TYPE_CMD &&
							device_bond_flag == false) 
						{	
							if(rx_payload.data[10] == CMD_OK ) // если получен ответ ОК
							{
								ok_retry_cnt = 0;
								nrf_gpio_pin_clear(LED_0); // выключим светодиод
								device_lost_packets=0;
								device_send_flag = false;
								lpm_flag = true;
							}
							else{
								// Если ОК не получен
								device_lost_packets++;
								if(ok_retry_cnt++ < 6)
								{
									lpm_flag = false;
									radio_set_channel(current_channel);
									device_load.device_freq_n 			= current_channel;
									device_load.device_lost_packs 	= device_lost_packets;
									radio_send_payload(&device_load);
								}
								else{
									ok_retry_cnt = 0;
									device_send_flag = false;
									lpm_flag = true;
								}
							}
						}
						else
						{
							device_send_flag = false;
							lpm_flag = true;
						}
					}
					else
					{
						device_send_flag = false;
						lpm_flag = true;
					}
				}
			}
			/***********************************************************************************************/
			/***********************************************************************************************/
			/***********************************************************************************************/
		
			if(device_send_flag)
			{
				lpm_flag = false;
				device_send_flag = false;
				nrf_gpio_pin_set(LED_0); // включим светодиод
				
				#if defined (NRF51822)
					VBAT = nrf_adc_read_vbat_mv();
				#elif defined (NRF52832)
					SAADC_Config();
					SAADC_StartMeasure();
					if(SAADC_DATAREADY_FLAG==true)
					{
						SAADC_DATAREADY_FLAG=false;
					}
					SAADC_Disable();
				#endif
				device_load.device_msg_type 		= MASSAGE_TYPE_TELE;
				device_load.device_type 				= DEVICE_TYPE_GERKON;
				device_load.device_id 					= DEVICE_ID;
				device_load.gateway_id 					= saved_gateway_id;
				device_load.device_vbat 				= VBAT;
				device_load.device_freq_n 			= current_channel;
				device_load.device_lost_packs 	= device_lost_packets;
				device_load.device_data_len		= sizeof(gerkon_state);
				memcpy(device_load.device_data, gerkon_state, sizeof(gerkon_state));
				radio_config(TX);
				radio_set_channel(current_channel);
				radio_send_payload(&device_load);
			}		
			/***********************************************************************************************/
			/***********************************************************************************************/
			/***********************************************************************************************/
			if(lpm_flag)
			{
				nrf_gpio_pin_clear(LED_0);
				retry_cnt = 0;
				
				nrf_esb_disable(); // останавливаем
				while(!nrf_esb_is_idle());
				NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
				
				// Уходим в спящий режим если ничего не приняли	
				clocks_set(XTAL_LF, true);
				clocks_set(XTAL_HF, false);
				
				// разрешаем прерывания от кнопки и геркона
				NVIC_EnableIRQ(GPIOTE_IRQn);
								
				//NRF_POWER->TASKS_LOWPWR = 1UL;
				
//				#ifdef NRF52
//				NRF_POWER->RAM[0].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[1].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[2].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[3].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[4].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[5].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[6].POWER = RAM_RETENTION_OFF;
//				NRF_POWER->RAM[7].POWER = RAM_RETENTION_OFF;
//				#endif //NRF52

//				NRF_POWER->SYSTEMOFF = 0x1;
//				(void) NRF_POWER->SYSTEMOFF;
//				while (true);
				// ждем события
				
				__WFE();
				__SEV();
				__WFE();
			}
    }
}

/*lint -restore */
