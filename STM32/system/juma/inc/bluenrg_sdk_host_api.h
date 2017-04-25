#ifndef _BLUENRG_SDK_HOST_API_H_
#define	_BLUENRG_SDK_HOST_API_H_

#include "cube_hal.h"
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"
#include <stdlib.h>

/*scan param struct*/
typedef struct
{
    uint8_t               type;                 /**< See @ref BLE_GAP_ADV_TYPES. */
    //ble_gap_addr_t       *p_peer_addr;          /**< For @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND mode only, known peer address. */
    uint8_t               fp;                   /**< Filter Policy, see @ref BLE_GAP_ADV_FILTER_POLICIES. */
    //ble_gap_whitelist_t  *p_whitelist;          /**< Pointer to whitelist, NULL if none is given. */
    uint16_t              scan_interval;             /**< Advertising interval between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s), see @ref BLE_GAP_ADV_INTERVALS.
                                                   - If type equals @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND, this parameter must be set to 0 for high duty cycle directed advertising.
                                                   - If type equals @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND, set @ref BLE_GAP_ADV_INTERVAL_MIN <= interval <= @ref BLE_GAP_ADV_INTERVAL_MAX for low duty cycle advertising.*/
    uint16_t              scan_window;              /**< Advertising timeout between 0x0001 and 0x3FFF in seconds, 0x0000 disables timeout. See also @ref BLE_GAP_ADV_TIMEOUT_VALUES. If type equals @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND, this parameter must be set to 0 for High duty cycle directed advertising. */
// ble_gap_adv_ch_mask_t channel_mask;         /**< Advertising channel mask. @see ble_gap_channel_mask_t for documentation. */
} ble_gap_scan_params_t;

/*Host*/
/*BLE scan param*/
void ble_host_set_scan_param(uint8_t* own_address, uint8_t tx_power_level, uint16_t scan_interval);
/*ble host scan*/
tBleStatus ble_host_start_scan(void);
/*ble host stop scan*/
tBleStatus ble_host_stop_scan(void);
/*BLE Host Connect*/
void ble_host_connect(tBDAddr bdaddr);
/*host discover char*/
void ble_host_discover_char(void* arg);
/*send data*/
tBleStatus ble_host_send(uint8_t type, uint32_t length, uint8_t* value);
/*receive data*/
void ble_host_on_message(uint8_t type, uint16_t length, uint8_t* value);
/*host connected*/
/*BLE On Connnection State*/
void ble_host_on_connect( void );

/*host found device*/
void ble_host_device_found( le_advertising_info* adv_data);

#endif /*_BLUENRG_SDK_HOST_API_H_*/





