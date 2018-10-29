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
 *
 * @defgroup ble_sdk_bluetooth_template_main main.c
 * @{
 * @ingroup bluetooth_template
 * @brief bluetooth_template main file.
 *
 * This file contains a template for creating a new application using Bluetooth Developper Studio generated code.
 * It has the code necessary to wakeup from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "service_if.h"
//#define NRF_LOG_MODULE_NAME "APP"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"

#if defined (NRF51822)
		#include "nrf51.h"
		#include "nrf51_bitfields.h"
#elif defined (NRF52832)
		#include "nrf52.h"
		#include "nrf52_bitfields.h"
#endif

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_rtc.h"

#include "service_if.h"
#include "ble_bas.h" 
#include "ble_meter.h" 
#include "ble_clock.h" 
#include "ble_wlan.h" 

#include "nrf52_adc.h"
#include "esp8266.h"
#include "FLASH.h"

#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

static FATFS fs;
static DIR dir;
static FILINFO fno;
static FIL file;
#define FILE_NAME   "LOG.TXT"

#define SDC_SCK_PIN     25  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    23  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    24  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      22  ///< SDC chip select (CS) pin.

FRESULT ff_result;
DSTATUS disk_state = STA_NOINIT;		
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

#define USER_RTC 2
#define COUNT_RTC_CC 0
#define TIMER_RTC_CC  1
#define COUNT_RTC_TICKS   (RTC_US_TO_TICKS(5000000ULL, RTC_DEFAULT_CONFIG_FREQUENCY)) // 5 sec
#define TIMER_RTC_TICKS    (RTC_US_TO_TICKS(100000ULL, RTC_DEFAULT_CONFIG_FREQUENCY)) // 0.1 sec
static nrf_drv_rtc_config_t const m_rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;
static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(USER_RTC);


uint8_t wlan_tcp_status = 0;
uint8_t get_request[256];
uint8_t sdc_log[256];
uint32_t counter1 = 0;
uint32_t counter2 = 0;

bool hot_water_notify_flag = false;
bool cold_water_notify_flag = false;
bool send_flag = false;
bool wlan_rdy_flag = false;

extern ble_bas_t    	m_bas; 
extern ble_wlan_t    	m_wlan; 
extern ble_meter_t    m_meter; 
extern ble_clock_t    m_clock; 


ble_bas_battery_level_t											m_ble_bas_battery_level;

ble_meter_hot_t						 									m_water_meter_hot_water_characteristic;
ble_meter_cold_t  													m_water_meter_cold_water_characteristic;

ble_wlan_action_t														m_wlan_action_characteristic;
ble_wlan_ssid_t															m_wlan_ssid_characteristic;
ble_wlan_pass_t															m_wlan_pass_characteristic;

ble_clock_hours_t														m_ble_clock_hours_characteristic;
ble_clock_minutes_t													m_ble_clock_minutes_characteristic;
ble_clock_seconds_t													m_ble_clock_seconds_characteristic;

extern uint16_t VBAT;
extern bool SAADC_DATAREADY_FLAG;
bool adc_measure_flag = false;

bool wlan_update_flag = false;
uint8_t SSID[16];
uint8_t PSWD[16];
uint16_t days = 0;
uint8_t hours = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t mills = 0; // 1 imp 100 ms
uint8_t timestamp[32] = "day: 0 time: 00:00:00";

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE             GATT_MTU_SIZE_DEFAULT                      /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "Water meter"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 200                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(100, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(200, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static nrf_ble_qwr_t                     m_qwr;                                     /**< Queued Writes structure.*/


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        SEGGER_RTT_WriteString(0, "GC completed\n");
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
    uint32_t err_code;
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    nrf_ble_qwr_init_t qwr_init;
    uint32_t           err_code;

    // Initialize Queued Write Module
    memset(&qwr_init, 0, sizeof(qwr_init));
    qwr_init.mem_buffer.len   = 0;
    qwr_init.mem_buffer.p_mem = NULL;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    err_code = bluetooth_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
					SEGGER_RTT_WriteString(0, "\r\nBLE: Fast Advertising\r\n");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            SEGGER_RTT_WriteString(0, "\r\nBLE: Idle Advertising\r\n");
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            SEGGER_RTT_WriteString(0, "\r\nBLE: Connected\r\n");
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            SEGGER_RTT_WriteString(0, "\r\nBLE: Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            SEGGER_RTT_WriteString(0, "\r\nBLE: GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            SEGGER_RTT_WriteString(0, "\r\nBLE: GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    bluetooth_on_ble_evt(p_ble_evt);
    nrf_ble_qwr_on_ble_evt(&m_qwr, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
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
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for system events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
//void bsp_event_handler(bsp_event_t event)
//{
//    uint32_t err_code;
//    switch (event)
//		{

//			
//        case BSP_EVENT_DISCONNECT:
//            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;

//        case BSP_EVENT_WHITELIST_OFF:
//            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//            {
//                err_code = ble_advertising_restart_without_whitelist();
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
//            break;

//        default:
//            break;
//    }
//}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            SEGGER_RTT_WriteString(0, "Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            SEGGER_RTT_WriteString(0, "Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n");/*,
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);*/
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
		
		nrf_gpio_pin_clear(17);
}

/**/
void GPIOTE_IRQHandler(void)
//void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		if(NRF_GPIOTE->EVENTS_IN[0] == 1)
		{	
			NRF_GPIOTE->EVENTS_IN[0] = 0;
			counter1++;

		}
		if(NRF_GPIOTE->EVENTS_IN[1] == 1)
		{			
			NRF_GPIOTE->EVENTS_IN[1] = 0;
			counter2++;
		}	
//		if( ((~NRF_GPIO->IN) >> 13) & 0x01)
//		{					
//				nrf_gpio_pin_clear(17);
//				SEGGER_RTT_WriteString(0, "Adverttising started!\r\n");
//				uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//				APP_ERROR_CHECK(err_code);
//		}
//		if( ((~NRF_GPIO->IN) >> 14) & 0x01 )
//		{						
//				nrf_gpio_pin_set(17);
//				SEGGER_RTT_WriteString(0, "Adverttising stopted!\r\n");
//				uint32_t err_code = sd_ble_gap_adv_stop();
//				APP_ERROR_CHECK(err_code);
//		}
}
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
		uint32_t err_code;
    bsp_event_t startup_event;
//    uint32_t err_code = bsp_init(NULL, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), bsp_event_handler);
//    APP_ERROR_CHECK(err_code);
//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
	
		nrf_gpio_cfg_output(17);
		nrf_gpio_cfg_output(18);
		nrf_gpio_cfg_output(19);
		nrf_gpio_cfg_output(20);
		nrf_gpio_pin_set(17);
		nrf_gpio_pin_set(18);
		nrf_gpio_pin_set(19);
		nrf_gpio_pin_set(20);
	
		nrf_gpio_cfg_input(3, GPIO_PIN_CNF_PULL_Pullup);
		nrf_gpio_cfg_input(4, GPIO_PIN_CNF_PULL_Pullup);
	
//		*(volatile uint32_t *)(NRF_GPIOTE_BASE + 0x600 + (4 * 0)) = 1;
//		*(volatile uint32_t *)(NRF_GPIOTE_BASE + 0x600 + (4 * 1)) = 1;
	
		NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
														3 << GPIOTE_CONFIG_PSEL_Pos												 |	
														GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos;
		
		NRF_GPIOTE->CONFIG[1] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
														4 << GPIOTE_CONFIG_PSEL_Pos												 |	
														GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos;	
	
		NRF_GPIOTE->EVENTS_IN[0] = 0;
		NRF_GPIOTE->EVENTS_IN[1] = 0;
	
		NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos | 
													 GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos ; 
		//NRF_GPIOTE->POWER = 	GPIOTE_POWER_POWER_Enabled << GPIOTE_POWER_POWER_Pos;
		NVIC_SetPriority(2, GPIOTE_IRQn);
		NVIC_DisableIRQ(GPIOTE_IRQn);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
/**/
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	ret_code_t err_code;
	
	switch(int_type)
	{
		case NRF_DRV_RTC_INT_COMPARE0:
			nrf_gpio_pin_toggle(18);
					
			send_flag = true;
			err_code = nrf_drv_rtc_cc_set(
					&m_rtc,
					COUNT_RTC_CC,
					(nrf_rtc_cc_get(m_rtc.p_reg, COUNT_RTC_CC) + COUNT_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
					true);
			APP_ERROR_CHECK(err_code);
			break;
		
		case NRF_DRV_RTC_INT_COMPARE1:
			nrf_gpio_pin_toggle(20);
		
			if(++mills == 10)
			{
				mills = 0;
				
				m_water_meter_hot_water_characteristic.hot = counter1;
				ble_meter_hot_set(&m_meter, &m_water_meter_hot_water_characteristic);
				if(hot_water_notify_flag){
					ble_meter_hot_notify(&m_meter, &m_water_meter_hot_water_characteristic);
				}
				
				m_water_meter_cold_water_characteristic.cold = counter2;
				ble_meter_cold_set(&m_meter, &m_water_meter_cold_water_characteristic);
				if(cold_water_notify_flag){
					ble_meter_cold_notify(&m_meter, &m_water_meter_cold_water_characteristic);
				}
				
				m_ble_clock_seconds_characteristic.seconds = seconds;
				ble_clock_seconds_set(&m_clock, &m_ble_clock_seconds_characteristic);
				
				m_ble_clock_minutes_characteristic.minutes = minutes;
				ble_clock_minutes_set(&m_clock, &m_ble_clock_minutes_characteristic);
				
				m_ble_clock_hours_characteristic.hours = hours;
				ble_clock_hours_set(&m_clock, &m_ble_clock_hours_characteristic);
				
				if(++seconds == 60)
				{
					seconds=0;
					adc_measure_flag = true;
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
			sprintf(timestamp, "day: %d time: %.2d:%.2d:%.2d", days, hours, minutes, seconds);
			err_code = nrf_drv_rtc_cc_set(
					&m_rtc,
					TIMER_RTC_CC,
					(nrf_rtc_cc_get(m_rtc.p_reg, TIMER_RTC_CC) + TIMER_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
					true);
			APP_ERROR_CHECK(err_code);
			break;
	}
}
/**/
static bool SDC_LOG_INIT(void)
{
		uint8_t msg[128];
	    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
      DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    SEGGER_RTT_WriteString(0, "SDC: Initializing disk 0...\r\n");
    for (uint32_t retries = 10; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        SEGGER_RTT_WriteString(0, "SDC: Disk initialization failed\r\n");
        return 0;
    }
		else
		{
			nrf_gpio_pin_clear(19);
		}
    
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    
		
		sprintf(msg, "SDC: Capacity: %d MB\r\n", capacity);
		SEGGER_RTT_WriteString(0, msg);

    SEGGER_RTT_WriteString(0, "SDC: Mounting...\r\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        SEGGER_RTT_WriteString(0, "SDC: Mount failed\r\n");
        return 0;
    }
		
		ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        SEGGER_RTT_WriteString(0, "SDC: Directory listing failed!\r\n");
        return 0;
    }
		
		do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            SEGGER_RTT_WriteString(0, "SDC: Directory read failed\r\n");
            return 0;
        }
        
        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
								sprintf(msg,	"SDC: <DIR> %s\r\n",(uint32_t)fno.fname);
								SEGGER_RTT_WriteString(0, msg);
            }
            else
            {
                sprintf(msg, "SDC: %.9lu  %s\r\n", fno.fsize, (uint32_t)fno.fname);
								SEGGER_RTT_WriteString(0, msg);
            }
        }
    }
    while (fno.fname[0]);
    SEGGER_RTT_WriteString(0, "\r\n");
		return 1;
}

/**/
static void SDC_LOG_CLEAR(void)
{
		ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_EXISTING);
    ff_result = f_write(&file, "", strlen("") - 1, NULL);
    (void) f_close(&file);
}
/**/
static void SDC_LOG(uint8_t* data)
{
    uint32_t bytes_written;
    ff_result = f_open(&file, FILE_NAME,  FA_READ | FA_WRITE | FA_OPEN_APPEND);
    ff_result = f_write(&file, data, strlen(data) - 1, (UINT *) &bytes_written);
	  if (ff_result == FR_OK)
    {
			nrf_gpio_pin_toggle(19);
    }
    (void) f_close(&file);
}
/**/
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
	SEGGER_RTT_WriteString(0, "RTC started!\r\n");
}
/**@brief Function for application main entry.
 */

int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	
		// READ SETTINGS FROM FLASH
		flash_read(FLASH_USER_START_ADDR, (uint32_t *)SSID, 4);
		flash_read(FLASH_USER_START_ADDR + 4, (uint32_t *)PSWD, 4);

	  // INITIALISE PEREPHERIALS
    buttons_leds_init(&erase_bonds);
		SAADC_Config();
		ESP8266_Serial_Config(115200UL);
//		SDC_LOG_INIT();
    timers_init();
					
		// INITIALISE BLE STACK
    ble_stack_init();
	  peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        SEGGER_RTT_WriteString(0, "BLE: Bonds erased!\r\n");
    }
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

    // START BLE
    application_timers_start();
    SEGGER_RTT_WriteString(0, "BLE: Start Advertising \r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
		// PREPARE WLAN SETTINGS
		if(strcmp((const char*)SSID, "ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿ")==0 && strcmp((const char*)PSWD, "ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿ")==0 &&
			 strcmp((const char*)SSID, "")==0 								&& strcmp((const char*)PSWD, "")==0)
		{
			SEGGER_RTT_WriteString(0, "WLAN: No SSID and PSWD in flash!\r\n");
			wlan_rdy_flag = false;
		}
		else
		{
			m_wlan_action_characteristic.action = 0;
			ble_wlan_action_set(&m_wlan, &m_wlan_action_characteristic);
			
			m_wlan_ssid_characteristic.ssid.p_str = SSID;
			m_wlan_ssid_characteristic.ssid.length = strlen(SSID);
			ble_wlan_ssid_set(&m_wlan, &m_wlan_ssid_characteristic);

			m_wlan_pass_characteristic.pass.p_str = PSWD;
			m_wlan_pass_characteristic.pass.length = strlen(PSWD);
			ble_wlan_pass_set(&m_wlan, &m_wlan_pass_characteristic);
			
			SEGGER_RTT_WriteString(0, (const char*)"SSID: ");
			SEGGER_RTT_WriteString(0, (const char*)SSID);
			SEGGER_RTT_WriteString(0, (const char*)"\r\nPASS: ");
			SEGGER_RTT_WriteString(0, (const char*)PSWD);
			SEGGER_RTT_WriteString(0, "\r\n");	
			wlan_rdy_flag = true;
		}
		
		// PREPARE COUNTER
		counter1=0;
		counter2=0;
		rtc_setup();
		NVIC_EnableIRQ(GPIOTE_IRQn);
		
		
		adc_measure_flag = true;
		// Main loop
    while(true)
    {
			if(wlan_update_flag)
			{
				// STOP WLAN IF WORKING
				wlan_rdy_flag = false;
				wlan_tcp_status = ESP8266_Session_Status();
				if(wlan_tcp_status == 2)
				{
					ESP8266_Wlan_Stop();
				}
				else if(wlan_tcp_status == 3)
				{
					ESP8266_Session_Close();
					ESP8266_Wlan_Stop();
				}	
				
				// WRITE SETTINGS IN FLASH
				SEGGER_RTT_WriteString(0, "WLAN: Update settings...\r\n");	
				sd_flash_page_erase(127);
				nrf_delay_ms(20);
				sd_flash_write(FLASH_USER_START_ADDR,(uint32_t *)SSID, 4);
				nrf_delay_ms(20);		
				sd_flash_write(FLASH_USER_START_ADDR + 4,(uint32_t *)PSWD, 4);
				nrf_delay_ms(20);
				
				// PRINTING
				SEGGER_RTT_WriteString(0, (const char*)"SSID: ");
				SEGGER_RTT_WriteString(0, (const char*)SSID);
				SEGGER_RTT_WriteString(0, (const char*)"\r\nPASS: ");
				SEGGER_RTT_WriteString(0, (const char*)PSWD);
				SEGGER_RTT_WriteString(0, "\r\n");	
				
				// UPDATE  CHARACTERISTICS
				m_wlan_action_characteristic.action = 0;
				ble_wlan_action_set(&m_wlan, &m_wlan_action_characteristic);
				
				m_wlan_ssid_characteristic.ssid.p_str = SSID;
				m_wlan_ssid_characteristic.ssid.length = strlen(SSID);
				ble_wlan_ssid_set(&m_wlan, &m_wlan_ssid_characteristic);
				
				m_wlan_pass_characteristic.pass.p_str = PSWD;
				m_wlan_pass_characteristic.pass.length = strlen(PSWD);
				ble_wlan_pass_set(&m_wlan, &m_wlan_pass_characteristic);
				
				// SET FLAGS
				wlan_update_flag = false;
				wlan_rdy_flag = true;
				
				// PRINTING
				SEGGER_RTT_WriteString(0, "WLAN: Settings updated!\r\n");	
			}
			if(adc_measure_flag && !wlan_update_flag)
			{
					// MEASURE BAT VOLTAGE
				  SAADC_Enable();
					SAADC_StartMeasure();
					while(!SAADC_DATAREADY_FLAG){};
					SAADC_DATAREADY_FLAG = false;
					SAADC_Disable();
					VBAT = (VBAT * 100) / 330 ;
					
					// PRINTING AND LOD TO uSD CARD
					sprintf(sdc_log, "\r\n%s VBAT=%d%%\r\n", timestamp, VBAT);
					SEGGER_RTT_WriteString(0, sdc_log);
					SDC_LOG(sdc_log);
					
					// UPDATE CHARACTERISTIC
					m_ble_bas_battery_level.level = VBAT;
					ble_bas_battery_level_send(&m_bas, &m_ble_bas_battery_level);
					
					// CLEAR MEASURE FLAG
					adc_measure_flag = false;
			}
			if(send_flag && !wlan_update_flag)
			{	
				// PRINTING COUNTERS AND LOG TO uSD CARD
				sprintf(sdc_log, "\r\n%s counter1=%d\r\n", timestamp, counter1);
				SEGGER_RTT_WriteString(0, sdc_log);
				SDC_LOG(sdc_log);

				sprintf(sdc_log, "%s counter2=%d\r\n\r\n", timestamp, counter2);
				SEGGER_RTT_WriteString(0, sdc_log);
				SDC_LOG(sdc_log);

				// SEND DATA TO SERVER
				if(wlan_rdy_flag)
				{
					if(ESP8266_Preinit())
					{
						sprintf(sdc_log, "\r\n%s WLAN Init ok!\r\n", timestamp);
						SDC_LOG(sdc_log);
						if(ESP8266_Wlan_Start(SSID, PSWD))
						{
							sprintf(sdc_log, "%s WLAN Started! IP: %s\r\n", timestamp, ESP8266_Get_IP());
							SDC_LOG(sdc_log);
							wlan_tcp_status = ESP8266_Session_Status();

							sprintf(sdc_log, "%s WLAN Status: %d\r\n", timestamp, wlan_tcp_status);
							SDC_LOG(sdc_log);
							
							if(wlan_tcp_status == 2 || wlan_tcp_status == 4)
							{
								sprintf(sdc_log, "%s WLAN Opening session!\r\n", timestamp);
								SDC_LOG(sdc_log);
								
								if(ESP8266_Session_Open("SSL", "192.168.1.252", 9443))
								{
									sprintf(sdc_log, "%s WLAN Session opened!\r\n", timestamp);
									SDC_LOG(sdc_log);
									
									wlan_tcp_status = ESP8266_Session_Status();
									if(wlan_tcp_status == 3)
									{
											sprintf(sdc_log, "%s WLAN Send data...\r\n", timestamp);
											SDC_LOG(sdc_log);
										
											memset(get_request, 0 , 256);
											sprintf(get_request, "/22dd4423c4a34bf5b989a6342cc691d6/update/V0?value=%d", counter1);
											if(ESP8266_GET_Req(get_request))
											{		
												sprintf(sdc_log, "%s WLAN Send data OK!\r\n", timestamp);
												SDC_LOG(sdc_log);
											}
											else
											{
												sprintf(sdc_log, "%s WLAN Send data ERROR!\r\n", timestamp);
												SDC_LOG(sdc_log);
											}
											
											
											sprintf(sdc_log, "%s WLAN Send data...\r\n", timestamp);
											SDC_LOG(sdc_log);
											
											memset(get_request, 0 , 256);
											sprintf(get_request, "/22dd4423c4a34bf5b989a6342cc691d6/update/V1?value=%d", counter2);
											if(ESP8266_GET_Req(get_request))
											{		
												sprintf(sdc_log, "%s WLAN Send data OK!\r\n", timestamp);
												SDC_LOG(sdc_log);
											}
											else
											{
												sprintf(sdc_log, "%s WLAN Send data ERROR!\r\n", timestamp);
												SDC_LOG(sdc_log);
											}
											sprintf(sdc_log, "%s WLAN Closing Session...\r\n", timestamp);
											SDC_LOG(sdc_log);
											if(ESP8266_Session_Close())
											{
												sprintf(sdc_log, "%s WLAN Session closed!\r\n", timestamp);
												SDC_LOG(sdc_log);
											}
											else
											{
												sprintf(sdc_log, "%s WLAN Session closing error!\r\n", timestamp);
												SDC_LOG(sdc_log);
											}
										}
									}
								}
							}
							
							sprintf(sdc_log, "%s WLAN Stoping...\r\n", timestamp);
							SDC_LOG(sdc_log);
							
							wlan_tcp_status = ESP8266_Session_Status();
							if(wlan_tcp_status != 5)
							{
								if(ESP8266_Wlan_Stop())
								{
									sprintf(sdc_log, "%s WLAN Stopted!\r\n", timestamp);
									SDC_LOG(sdc_log);
								}
								else
								{
									sprintf(sdc_log, "%s WLAN Stop error!\r\n", timestamp);
									SDC_LOG(sdc_log);
								}	
							}
					}					
				}
				// CLEAR SEND FLAG
				send_flag = !send_flag;
			}
			// SLEEP MODE
			power_manage();
    }
}

void HardFault_Handler(void)
{
	SEGGER_RTT_WriteString(0, (const char*)"\r\nHard fault!\r\n");
	sprintf(sdc_log, "%s Hard fault!\r\n", timestamp);
	SDC_LOG(sdc_log);
}
/**
 * @}
 */
