#ifndef _BLUENRG_SDK_API_H_
#define _BLUENRG_SDK_API_H_


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

void sleep_flag_set(uint8_t flag);

#define run_after_delay(func, args, delay)          run_at_time((func), (args), current_time() + (delay))

typedef void (*function_t)(void* args);

/*Ble State*/
#define BLE_CONNECTABLE		1
#define BLE_NOCONNECTABLE	0

/*connected*/
#define CONNECT_DEVICE_ENABLE  0x03
/*scan interval*/
#define SCAN_P (0x4000)
#define SCAN_L (0x4000)

/**
* @brief Supervision timeout, arg in msec.
*/
#define SUPERV_TIMEOUT (60)
/**
* @brief Connection period, arg in msec.
*/
#define CONN_P(x) ((int)((x)/1.25f))

/**
* @brief Connection length, arg in msec.
*/
#define CONN_L(x) ((int)((x)/0.625f))

#define CONN_P1     (CONN_P(50))
#define CONN_P2     (CONN_P(50))

#define CONN_L1     (CONN_L(1250))
#define CONN_L2     (CONN_L(1250))

typedef struct scan_device_found {
    uint8_t		bdaddr_type;  /**< Type of the peer address (@ref PUBLIC_ADDR, @ref RANDOM_ADDR). */
    tBDAddr	 bdaddr;       /**< Address of the peer device found during scanning. */
    uint8_t		local_name_len;  /**< Length of advertising or scan response data. */
    uint8_t  local_name[VARIABLE_SIZE];
    uint8_t		uuid_type; /**< Advertising or scan response data + RSSI. RSSI is last octect (signed integer). */
    uint8_t  uuid[VARIABLE_SIZE];
    int8_t RSSI;
} scan_device_found_info;

typedef enum {
   SCAN_FILTER_DUPLICATE_DISABLE = 0, /**< scan filter duplicate disable. */
   SCAN_FILTER_DUPLICATE_ENABLE = 1  /**< scan filter duplicate enable. */
} SCAN_FilterDuplicateType;

typedef enum {
    SCAN_PASSIVE = 0,  /**< SERVER is for Peripheral role. */
    SCAN_ACTIVE = 1   /**< CLIENT is for Central role. */
} SCAN_Type;

typedef enum {
    CLIENT = 0, /**< CLIENT is for Central role. */
    SERVER = 1  /**< SERVER is for Peripheral role. */
} BLE_RoleTypeDef;


/*Ble Connection State*/
extern volatile uint8_t Ble_conn_state;

uint32_t current_time(void);
void run_when_idle(function_t func, void* args);
void run_at_time(function_t func, void* args, uint32_t time);

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void);
/*connection complete callback*/
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
/*gatt notification call back*/
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value);

void on_ready(void);
/*Init BLUENrg, HCI, Add Service*/
tBleStatus ble_init_bluenrg(void);
/*BLE Set Tx Power*/
tBleStatus ble_set_tx_power(uint8_t level);
/*Add Adv Address*/
tBleStatus ble_address(uint8_t* advaddress);

/*Set Adv Namee*/
tBleStatus ble_device_set_name(const char* new_device_name);
/*Config Adv Interval (min_adv_interval > 32*0.625)*/
void ble_device_set_advertising_interval(uint16_t interval);
/*Start to adv*/
tBleStatus ble_device_start_advertising(void);
/*set adv param*/
void ble_set_adv_param(char* adv_name, uint8_t*adv_address, uint8_t tx_power_level, uint16_t adv_interval);
/*Stop Adv*/
tBleStatus ble_device_stop_advertising(void);
/*Tx data(Notify)*/
tBleStatus ble_device_send(uint8_t type, uint32_t length, uint8_t* value);
/*To Disconnect Ble Connection*/
tBleStatus ble_user_disconnect_device(void);
/*Rx Data(write / write without responce)*/
void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value);
/*BLE On Connnection State*/
void ble_device_on_connect( void );
/*BLE On Disconnection State*/
void ble_device_on_disconnect(uint8_t reason);

void Error_Handler(void);

#endif //_BLUENRG_SDK_API_H_




