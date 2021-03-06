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
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "boards.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

#include "aes.h"
#define ECB 1
#include "nrf_drv_rtc.h"

uint32_t seconds = 0;

#define USER_RTC 						2
#define COUNT_RTC_CC 				0
#define TIMER_RTC_CC  			1
#define COUNT_RTC_TICKS   	(RTC_US_TO_TICKS(1000000ULL, RTC_DEFAULT_CONFIG_FREQUENCY)) // 1 sec
#define TIMER_RTC_TICKS    	(RTC_US_TO_TICKS(1000000ULL, RTC_DEFAULT_CONFIG_FREQUENCY)) // 1 sec
static nrf_drv_rtc_config_t const m_rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;
static nrf_drv_rtc_t 				const m_rtc = NRF_DRV_RTC_INSTANCE(USER_RTC);


#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEVICE_NAME											"GERKON"
#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */


#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf52_adc.h"
#define LED_0 	9
#define GERKON 	8
#define BUTTON 	10

extern uint16_t VBAT;	
extern bool SAADC_DATAREADY_FLAG;

uint8_t gerkon_state = 0;
bool adc_ready = false;
bool data_ready = false;
bool device_pwr_up = false;

static	ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
uint32_t uptime = 0;		
uint8_t my_data[16] = 
{
	0x01, //device type 01 -GERKON 
	
	0x00, 
	0x01, //2 bytes device ID
	
	0x00,	//
	0x00, // 2 bytes VBat
	
	0x00,  //gerkon state
	
	0x00,
	0x00,
	0x00,
	0x00, // 4 bytes uptime
	
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF
};
uint8_t enc_data[16];
uint8_t dec_data[16];
uint8_t key[16] = "ufanet1234567890"; 
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(uint8_t *data, uint8_t len)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
		
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data =  data;
    manuf_specific_data.data.size   = len;
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
		ble_gap_addr_t     peer_addr = {1, BLE_GAP_ADDR_TYPE_PUBLIC,{0x39,0xA4,0xED,0x34,0x77,0xAC}};
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}

static void advertising_stop(void)
{
    uint32_t                err_code;
		err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);																			
} 

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;
		//sd_ble_gap_tx_power_set(4);
		//sd_power_dcdc_mode_set(1);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)DEVICE_NAME,strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);																			
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	ret_code_t err_code;
	
	switch(int_type)
	{
		case NRF_DRV_RTC_INT_COMPARE0:
				uptime++;
				my_data[6]= (uint8_t)(uptime>>24);
				my_data[7]= (uint8_t)(uptime>>16);
				my_data[8]= (uint8_t)(uptime>>8);
				my_data[9]= (uint8_t)uptime;
		
				if(seconds++ == 3600)
				{
					data_ready = true;
					seconds=0;
				}
				
				err_code = nrf_drv_rtc_cc_set(
					&m_rtc,
					COUNT_RTC_CC,
					(nrf_rtc_cc_get(m_rtc.p_reg, COUNT_RTC_CC) + COUNT_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
					true);
				APP_ERROR_CHECK(err_code);
			break;
		
		case NRF_DRV_RTC_INT_COMPARE1:
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
//	err_code = nrf_drv_rtc_cc_set(&m_rtc, TIMER_RTC_CC, TIMER_RTC_TICKS, true);
//	APP_ERROR_CHECK(err_code);
	nrf_drv_rtc_enable(&m_rtc);
	//NRF_LOG_INFO("RTC started!\r\n");
}

void GPIOTE_IRQHandler(void)
{
	if(NRF_GPIOTE->EVENTS_PORT)
	{
		NVIC_DisableIRQ(GPIOTE_IRQn);
		NRF_GPIOTE->EVENTS_PORT = 0;
		if( ((~NRF_GPIO->IN) >> BUTTON) & 0x01)
		{		
				device_pwr_up = true;		
		}	
		else if( ((~NRF_GPIO->IN) >> GERKON) & 0x01 )
		{			
				nrf_gpio_cfg_sense_input(GERKON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos,
																 NRF_GPIO_PIN_SENSE_HIGH); 	// Interrupt low-to-high for close
				gerkon_state = 1;
				my_data[5] = gerkon_state;
			
		}
		else if( !(((~NRF_GPIO->IN) >> GERKON) & 0x01) )
		{		
				nrf_gpio_cfg_sense_input(GERKON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos, 
																 NRF_GPIO_PIN_SENSE_LOW); 	// Interrupt high-to-low for close
				gerkon_state = 0;
				my_data[5] = gerkon_state;
		}
		data_ready = true;
	}
}

static void buttons_leds_init(void)
{
		NVIC_DisableIRQ(GPIOTE_IRQn);	 
		NVIC_SetPriority(2, GPIOTE_IRQn);
		NRF_GPIOTE->EVENTS_PORT = 0;
		// Disable NFC pins
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		NRF_UICR->NFCPINS = UICR_NFCPINS_PROTECT_Disabled << UICR_CUSTOMER_CUSTOMER_Pos;
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren<< NVMC_CONFIG_WEN_Pos;
		// ��������� - LED
		nrf_gpio_cfg_output(LED_0); 
		nrf_gpio_pin_clear(LED_0); 
		// ������ - Button
		nrf_gpio_cfg_sense_input(BUTTON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos, NRF_GPIO_PIN_SENSE_LOW); 
		// ������ - Gerkon
		nrf_gpio_cfg_sense_input(GERKON, GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos, NRF_GPIO_PIN_SENSE_LOW); 
		NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;  
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
		ble_gap_addr_t adr;
    uint32_t err_code;
	
    // Peripherial nitialize.
		buttons_leds_init();
		SAADC_Config();	
		rtc_setup();
	
		// BLE Init
    ble_stack_init();
		gap_params_init();
		sd_ble_gap_addr_get(&adr);
	
    // Start execution.
		nrf_gpio_pin_set(LED_0);
		data_ready= true;
	
    // Enter main loop.
    for (;; )
    {
				if(data_ready)
				{
					data_ready = false;
					for(int i =0; i<5; i++)
					{
						SAADC_Enable();
						SAADC_StartMeasure();
						while(!SAADC_DATAREADY_FLAG);
						SAADC_DATAREADY_FLAG=false;
						SAADC_Disable();
					}
					my_data[3] = (uint8_t)(VBAT>>8); 
					my_data[4] = (uint8_t)(VBAT);

					AES128_ECB_encrypt(my_data, key, enc_data);
					//AES128_ECB_decrypt(enc_data, key, dec_data);
					advertising_init(enc_data, 16);
					advertising_start();
					nrf_gpio_pin_set(LED_0);
					nrf_delay_ms(1500);
					advertising_stop();
				}
				
				nrf_gpio_pin_clear(LED_0);
				NVIC_EnableIRQ(GPIOTE_IRQn);
				power_manage();
    }
}


/**
 * @}
 */
