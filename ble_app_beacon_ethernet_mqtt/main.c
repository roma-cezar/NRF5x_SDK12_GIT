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

#include "main.h"
#include "MQTTClient.h"

static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

APP_TIMER_DEF(m_app_timer_id); 
/***************************************************************************************************/
W5100_CFG my_cfg = {
	{0x8A, 0x26, 0xF4, 0x1A, 0xD0, 0x93},		// MAC address
	{192, 168, 1, 3},						// IP address
	{255, 255, 255, 0},							// Subnet mask
	{192, 168, 1, 1},							// Gateway

//	{192, 168, 1, 120}, 
	{192, 168, 1, 127},
//	SOCKET_PORT 
	MQTT_PORT
};
extern unsigned char debug_str[64];
extern uint8_t destip[4];
extern uint16_t destport;

extern uint8_t data_buf_rx[255];
extern uint8_t data_buf_tx[255];

uint8_t ip[4];
uint8_t sn[4];
uint8_t gw[4];
uint8_t mac[6];

uint8_t mode = 1; //0 - DHCP, 1 - no DHCP
uint8_t my_socket = 0;
uint8_t err;

uint8_t tcpdata[] = {0x01, 0x02, 0x03, 0x0A, 0x6B, 0x0C};

uint8_t counter = 0;
/***************************************************************************************************/

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
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
		ble_advdata_t srdata;
    uint8_t       flags = (BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE | BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
		advdata.name_type 							= BLE_ADVDATA_NO_NAME;
    advdata.flags                 	= flags;
    advdata.p_manuf_specific_data 	= &manuf_specific_data;
		    
		
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
	
		err_code = sd_ble_gap_tx_power_set(TX_POWER);
    APP_ERROR_CHECK(err_code);
	
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
    //err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    //APP_ERROR_CHECK(err_code);																			
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
	
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
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
/**/
static void timerhandler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	uint32_t err_code;
	
	unsigned char p[16];
	
	nrf_gpio_pin_toggle(LED2);
	counter++;
	
	sprintf(p, "CNT=%d\0", counter);
	//tcpclient_sendbytes(my_socket, tcpdata, 6);
	//MQTTClient_Connect(my_socket, destip);
	//MQTTClient_Open_session(my_socket, "123456a");
  MQTTClient_Publish(my_socket, "nrf", p);
	//MQTTClient_Disconnect(my_socket);
}
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    // Create timers.
    uint32_t err_code;
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timerhandler);
    APP_ERROR_CHECK(err_code);
}
static void application_timers_start(void)
{
    //YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;
    err_code = app_timer_start(m_app_timer_id, APP_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

void ethernet_config(void)
{
	Serial_Print((uint8_t*)"\r\nConfig static IP mode...\r\n");

	iinchip_init();
	nrf_delay_ms(1000);
	sysinit(1, 1);
	nrf_delay_ms(1000);

	setSIPR(my_cfg.ip_addr);
	setGAR( my_cfg.gtw_addr);
	setSUBR(my_cfg.sub_mask);
	setSHAR(my_cfg.mac_addr);
	nrf_delay_ms(100);

	getSIPR(ip);
	getSUBR(sn);
	getGAR(gw);
	getSHAR(mac);

	memcpy(destip, my_cfg.remip, 4);
	destport = my_cfg.remport;
	
	sprintf(debug_str, "\r\nMY IP: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
	Serial_Print((uint8_t*)debug_str);

	sprintf(debug_str, "SUB NET: %d.%d.%d.%d\r\n", sn[0], sn[1], sn[2], sn[3]);
	Serial_Print((uint8_t*)debug_str);

	sprintf(debug_str, "GW: %d.%d.%d.%d\r\n", gw[0], gw[1], gw[2], gw[3]);
	Serial_Print((uint8_t*)debug_str);

	sprintf(debug_str, "MAC: %2X:%2X:%2X:%2X:%2X:%2X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	Serial_Print((uint8_t*)debug_str);	

	Serial_Print((uint8_t*)"\r\nW5100 configured!\r\n");
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
	uint32_t err_code;
	// Initialize.
	nrf_gpio_cfg_output(LED2);
	SPI_Init();
	timers_init();
	// memory set at 1024 bytes for flash data variables
	Serial_Config(115200);
	
	if(*my_cfg.ip_addr==NULL || *my_cfg.sub_mask==NULL || *my_cfg.gtw_addr==NULL || *my_cfg.remip==NULL || my_cfg.remport==NULL)
	{
		Serial_Print((uint8_t*)"\r\nConfigure Net settings!\r\n");
		GetIP((uint8_t*)"\r\nEnter IP address: ", 	my_cfg.ip_addr);
		GetSN((uint8_t*)"\r\nEnter subnet mask: ", my_cfg.sub_mask);
		GetGW((uint8_t*)"\r\nEnter gateway: ", 		my_cfg.gtw_addr);
		// TCP server parameters
		GetHOST((uint8_t*)"\r\nEnter remote server IP address: ", my_cfg.remip);
		GetPORT((uint8_t*)"\r\nEnter remote server port: ", &my_cfg.remport);	
	}
	ethernet_config();

	MQTTClient_Connect(my_socket, destip, destport);
	nrf_delay_ms(1000);
	MQTTClient_Open_session(my_socket, "123456a", "Roman1989!", "Roman1989!");
  //MQTTClient_Publish(my_socket, "nrf", "123");
	
	// BLE beacon setiings
	err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
	ble_stack_init();
	gap_params_init();
	advertising_init();
	// Start execution.
	advertising_start();
	application_timers_start();
	// Enter main loop.
	for(;;)
	{
		power_manage();
		
		if(CheckCmd("-netcfg"))
		{
			FlushRx();
			tcpclient_close(my_socket);
			Serial_Print((uint8_t*)"\r\nConfigure Net settings!\r\n");
			GetIP((uint8_t*)"\r\nEnter IP address: ", 	my_cfg.ip_addr);
			GetSN((uint8_t*)"\r\nEnter subnet mask: ", my_cfg.sub_mask);
			GetGW((uint8_t*)"\r\nEnter gateway: ", 		my_cfg.gtw_addr);
			ethernet_config();
		}
		else if(CheckCmd("-tcpcfg"))
		{
			FlushRx();
			tcpclient_close(my_socket);
			// TCP server parameters
			GetHOST((uint8_t*)"\r\nEnter remote server IP address: ", my_cfg.remip);
			GetPORT((uint8_t*)"\r\nEnter remote server port: ", &my_cfg.remport);	
			Serial_Print((uint8_t*)"\r\n");
			ethernet_config();
		}

	}
}
/**
 * @}
 */
