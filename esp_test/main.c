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

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"


#include "nrf_drv_rtc.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "esp8266.h"
#include "MQTTClient.h"
/**
 * @brief Function for application main entry.
 */

m_WLAN 		WLAN;
m_MQTT    MQTT;
extern  	m_SOCKET 	SOCKET;
extern uint8_t my_ip[];
extern uint8_t my_mac[];
bool send_flag = false;
uint16_t days = 0;
uint8_t hours = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t mills = 0; // 1 imp 100 ms
uint8_t timestamp[32] = "day: 0 time: 00:00:00.00";


 
#define USER_RTC 						2
#define COUNT_RTC_CC 				0
#define TIMER_RTC_CC  			1
#define COUNT_RTC_TICKS   	(RTC_US_TO_TICKS(10000000ULL, RTC_DEFAULT_CONFIG_FREQUENCY)) // 60 sec
#define TIMER_RTC_TICKS    	(RTC_US_TO_TICKS(100000ULL, RTC_DEFAULT_CONFIG_FREQUENCY)) // 0.1 sec
static nrf_drv_rtc_config_t const m_rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;
static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(USER_RTC);

uint8_t data[] = "1234567890123456789012345678901234567890qwerty!\r\n";
																	
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	ret_code_t err_code;
	
	switch(int_type)
	{
		case NRF_DRV_RTC_INT_COMPARE0:
			
			bsp_board_led_invert(2);
			send_flag = true;
			err_code = nrf_drv_rtc_cc_set(
					&m_rtc,
					COUNT_RTC_CC,
					(nrf_rtc_cc_get(m_rtc.p_reg, COUNT_RTC_CC) + COUNT_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
					true);
			APP_ERROR_CHECK(err_code);
			break;
		
		case NRF_DRV_RTC_INT_COMPARE1:
			if(++mills == 10)
			{
				mills = 0;
				if(++seconds == 60)
				{
					seconds=0;
					if(++minutes == 60)
					{
						minutes = 0;
						if(++hours == 24)
						{
							days++;
							hours = 0;
						}
					}
				}
			}
			sprintf(timestamp, "day: %d time: %.2d:%.2d:%.2d.%.2d", days, hours, minutes, seconds, mills);
			err_code = nrf_drv_rtc_cc_set(
					&m_rtc,
					TIMER_RTC_CC,
					(nrf_rtc_cc_get(m_rtc.p_reg, TIMER_RTC_CC) + TIMER_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
					true);
			APP_ERROR_CHECK(err_code);
			break;
	}
}

 static void rtc_setup(void)
{
	ret_code_t err_code;
	err_code = nrf_drv_rtc_init(&m_rtc, &m_rtc_config, rtc_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_rtc_cc_set(&m_rtc, COUNT_RTC_CC, COUNT_RTC_TICKS, true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_rtc_cc_set(&m_rtc, TIMER_RTC_CC, TIMER_RTC_TICKS, true);
	APP_ERROR_CHECK(err_code);
	nrf_drv_rtc_enable(&m_rtc);
	#ifdef ESP_DEBUG
	SEGGER_RTT_WriteString(0, "\r\nRTC started!\r\n");
	#endif
}

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void power_manage(void)
{
	__SEV();
	__WFE();
	__WFE();
}

int main(void)
{
    /* Configure board. */
		lfclk_config();
    bsp_board_leds_init();
		rtc_setup();
	
		WLAN.baudrate = 115200UL;
		WLAN.ssid = "dd-wrt";
		WLAN.psk = "bora-bora04";
		WLAN.wlan_mode = WLAN_MODE_STA;
		WLAN.ip_mode = IP_MODE_DHCP;
		
		MQTT.id 	= "As56yhEE01";
		MQTT.host = "m14.cloudmqtt.com";
		MQTT.port = 10578;
		MQTT.user = "ioibvutn";
		MQTT.pswd = "lzbHSKesAlEi";
		MQTT.topic	= "/metka";
		MQTT.timeout	= 10000;
		MQTT.secure		= false;
		// Get dinamic IP and store it as static IP
		bsp_board_led_invert(0); // pwr on
		if(ESP8266_Preinit(&WLAN))
		{
				if(ESP8266_Wlan_Start(&WLAN))
				{	
					memcpy(WLAN.ip, 	ESP8266_Get_IP(), 	16);
					memcpy(WLAN.mac, 	ESP8266_Get_MAC(), 	17);
					ESP8266_Ping(MQTT.host, 3);
					ESP8266_Wlan_Stop();
				}
				WLAN.ip_mode = IP_MODE_STATIC;
				ESP8266_Deinit();
		}
		bsp_board_led_invert(0); // pwr on

    while (true)
    {
			if(send_flag)
			{
				if(ESP8266_Preinit(&WLAN))
				{
					if(ESP8266_Wlan_Start(&WLAN))
					{		
							if(MQTTClient_Connect(&MQTT))
							{
								if(MQTTClient_Open_session(&MQTT))
								{
									MQTTClient_Publish(MQTT.topic, "Hello!\r\n");
								}
								MQTTClient_Disconnect();
							}

						ESP8266_Wlan_Stop();
					}
					ESP8266_Deinit();
				}
				bsp_board_led_invert(2);
				send_flag = false;
			}

			power_manage();
    }
}

/**
 *@}
 **/
