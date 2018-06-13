/* This file was generated by plugin 'Nordic Semiconductor nRF5x v.1.2.2' (BDS version 1.1.3139.0) */

#ifndef BLE_WLAN_H__
#define BLE_WLAN_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_bds.h"



/**@brief WLAN event type. */
typedef enum
{ 
    BLE_WLAN_WLAN_SSID_EVT_NOTIFICATION_ENABLED,  /**< WLAN SSID value notification enabled event. */
    BLE_WLAN_WLAN_SSID_EVT_NOTIFICATION_DISABLED, /**< WLAN SSID value notification disabled event. */
    BLE_WLAN_WLAN_SSID_EVT_WRITE, /**< WLAN SSID write event. */
    BLE_WLAN_WLAN_PASS_EVT_NOTIFICATION_ENABLED,  /**< WLAN PASS value notification enabled event. */
    BLE_WLAN_WLAN_PASS_EVT_NOTIFICATION_DISABLED, /**< WLAN PASS value notification disabled event. */
    BLE_WLAN_WLAN_PASS_EVT_WRITE, /**< WLAN PASS write event. */
    BLE_WLAN_WLAN_ACTION_EVT_NOTIFICATION_ENABLED,  /**< WLAN ACTION value notification enabled event. */
    BLE_WLAN_WLAN_ACTION_EVT_NOTIFICATION_DISABLED, /**< WLAN ACTION value notification disabled event. */
    BLE_WLAN_WLAN_ACTION_EVT_WRITE, /**< WLAN ACTION write event. */
} ble_wlan_evt_type_t;

// Forward declaration of the ble_wlan_t type.
typedef struct ble_wlan_s ble_wlan_t;








/**@brief WLAN SSID structure. */
typedef struct
{
    ble_srv_utf8_str_t ssid;
} ble_wlan_wlan_ssid_t;
/**@brief WLAN PASS structure. */
typedef struct
{
    ble_srv_utf8_str_t pass;
} ble_wlan_wlan_pass_t;
/**@brief WLAN ACTION structure. */
typedef struct
{
    uint8_t action;
} ble_wlan_wlan_action_t;

/**@brief WLAN Service event. */
typedef struct
{
    ble_wlan_evt_type_t evt_type;    /**< Type of event. */
    union {
        uint16_t cccd_value; /**< Holds decoded data in Notify and Indicate event handler. */
        ble_wlan_wlan_ssid_t wlan_ssid; /**< Holds decoded data in Write event handler. */
        ble_wlan_wlan_pass_t wlan_pass; /**< Holds decoded data in Write event handler. */
        ble_wlan_wlan_action_t wlan_action; /**< Holds decoded data in Write event handler. */
    } params;
} ble_wlan_evt_t;

/**@brief WLAN Service event handler type. */
typedef void (*ble_wlan_evt_handler_t) (ble_wlan_t * p_wlan, ble_wlan_evt_t * p_evt);

/**@brief WLAN Service init structure. This contains all options and data needed for initialization of the service */
typedef struct
{
    ble_wlan_evt_handler_t     evt_handler; /**< Event handler to be called for handling events in the WLAN Service. */
    ble_wlan_wlan_ssid_t ble_wlan_wlan_ssid_initial_value; /**< If not NULL, initial value of the WLAN SSID characteristic. */ 
    ble_wlan_wlan_pass_t ble_wlan_wlan_pass_initial_value; /**< If not NULL, initial value of the WLAN PASS characteristic. */ 
    ble_wlan_wlan_action_t ble_wlan_wlan_action_initial_value; /**< If not NULL, initial value of the WLAN ACTION characteristic. */ 
} ble_wlan_init_t;

/**@brief WLAN Service structure. This contains various status information for the service.*/
struct ble_wlan_s
{
    ble_wlan_evt_handler_t evt_handler; /**< Event handler to be called for handling events in the WLAN Service. */
    uint16_t service_handle; /**< Handle of WLAN Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t wlan_ssid_handles; /**< Handles related to the WLAN SSID characteristic. */
    ble_gatts_char_handles_t wlan_pass_handles; /**< Handles related to the WLAN PASS characteristic. */
    ble_gatts_char_handles_t wlan_action_handles; /**< Handles related to the WLAN ACTION characteristic. */
    uint16_t conn_handle; /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
};

/**@brief Function for initializing the WLAN.
 *
 * @param[out]  p_wlan       WLAN Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_wlan_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_wlan_init(ble_wlan_t * p_wlan, const ble_wlan_init_t * p_wlan_init);

/**@brief Function for handling the Application's BLE Stack events.*/
void ble_wlan_on_ble_evt(ble_wlan_t * p_wlan, ble_evt_t * p_ble_evt);

/**@brief Function for setting the WLAN SSID.
 *
 * @details Sets a new value of the WLAN SSID characteristic. The new value will be sent
 *          to the client the next time the client reads the WLAN SSID characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_wlan                 WLAN Service structure.
 * @param[in]   p_wlan_ssid  New WLAN SSID.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_wlan_wlan_ssid_set(ble_wlan_t * p_wlan, ble_wlan_wlan_ssid_t * p_wlan_ssid);

/**@brief Function for setting the WLAN PASS.
 *
 * @details Sets a new value of the WLAN PASS characteristic. The new value will be sent
 *          to the client the next time the client reads the WLAN PASS characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_wlan                 WLAN Service structure.
 * @param[in]   p_wlan_pass  New WLAN PASS.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_wlan_wlan_pass_set(ble_wlan_t * p_wlan, ble_wlan_wlan_pass_t * p_wlan_pass);

/**@brief Function for setting the WLAN ACTION.
 *
 * @details Sets a new value of the WLAN ACTION characteristic. The new value will be sent
 *          to the client the next time the client reads the WLAN ACTION characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_wlan                 WLAN Service structure.
 * @param[in]   p_wlan_action  New WLAN ACTION.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_wlan_wlan_action_set(ble_wlan_t * p_wlan, ble_wlan_wlan_action_t * p_wlan_action);

#endif //_BLE_WLAN_H__
