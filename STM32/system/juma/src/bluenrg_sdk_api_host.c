#include "bluenrg_sdk_host_api.h"
#include "bluenrg_sdk_api.h"

#if NO_PRINTF
#define printf(...)
#endif
extern BLE_RoleTypeDef BLE_Role;
extern uint16_t connection_handle;

ble_gap_scan_params_t host_scan_param;
scan_device_found_info device_info;

uint16_t write_handle;
uint16_t notify_read_handle;
uint16_t write_without_rsp_handle;
uint16_t notify_handle;

volatile int host_connected = FALSE;
volatile uint8_t host_notification_enabled = FALSE;
volatile uint8_t start_read_write_char_handle = FALSE;
volatile uint8_t start_read_notify_read_char_handle = FALSE;
volatile uint8_t start_read_write_without_rsp_char_handle = FALSE;
volatile uint8_t start_read_notify_char_handle = FALSE;
volatile uint8_t end_read_write_char_handle = FALSE;
volatile uint8_t end_read_notify_read_char_handle = FALSE;
volatile uint8_t end_read_write_without_rsp_char_handle = FALSE;
volatile uint8_t end_read_notify_char_handle = FALSE;

uint8_t host_connect_init_flag = FALSE;

/*Host*/
__weak void ble_host_on_device_info(scan_device_found_info device_info) {}
__weak void ble_host_on_message(uint8_t type, uint16_t length, uint8_t* value) {}
__weak void ble_host_on_connect( void ) {}
 
/*Host*/
static void read_write_char_handle(void);
static void read_notify_read_char_handle(void);
static void read_write_without_rsp_char_handle(void);
static void read_notify_char_handle(void);
static void enableNotification(void);
 
/*scan param*/
void ble_host_set_scan_param(uint8_t* own_address, uint8_t tx_power_level, uint16_t scan_interval)
{

    /*set address*/
    ble_address(own_address);
    /*Gatt And Gap Init*/
    ble_init_bluenrg();
    /*Set Tx Power Level*/
    ble_set_tx_power(tx_power_level);
    /*scan_interval scan window*/
    host_scan_param.scan_interval = scan_interval;
    host_scan_param.scan_window = scan_interval;
    host_scan_param.fp = SCAN_FILTER_DUPLICATE_DISABLE;

}

/*host scan device*/
tBleStatus ble_host_start_scan(void)
{
    tBleStatus ret;
    SCAN_Type scan_type = SCAN_ACTIVE;

    ret = aci_gap_start_general_conn_establish_proc(scan_type, SCAN_P,  SCAN_L, PUBLIC_ADDR, host_scan_param.fp);
    if(ret != BLE_STATUS_SUCCESS) {
        return BLE_STATUS_ERROR;
    }
    printf("start scanning \n\r");

    return 0;
}

/*host stop scan*/
tBleStatus ble_host_stop_scan(void)
{
    aci_gap_terminate_gap_procedure(GAP_GENERAL_DISCOVERY_PROC);
 
    return 0;
}

void ble_host_device_found( le_advertising_info* adv_data)
{
    uint8_t i;

    i = 25;
    while(adv_data->data_RSSI[i] == 0) {
        i--;
    }
    /*RSSI*/
    device_info.RSSI = adv_data->data_RSSI[i] - 255;
    /*bdaddr type*/
    device_info.bdaddr_type = adv_data->bdaddr_type;
    /*bdaddr*/
    memcpy(device_info.bdaddr, adv_data->bdaddr, 6);
    /*local name length*/
    device_info.local_name_len = adv_data->data_RSSI[3];
    /*local name*/
    memcpy(device_info.local_name, (adv_data->data_RSSI)+4, device_info.local_name_len);

    ble_host_on_device_info(device_info);
}


/*host creat connection*/
void ble_host_connect(tBDAddr bdaddr)
{
    if(host_connect_init_flag == FALSE){
       tBleStatus ret;
        printf("Client Create Connection\n");
        BSP_LED_On(LED0);
        /*
          Scan_Interval, Scan_Window, Peer_Address_Type, Peer_Address, Own_Address_Type, Conn_Interval_Min,
          Conn_Interval_Max, Conn_Latency, Supervision_Timeout, Conn_Len_Min, Conn_Len_Max
          */
        ret = aci_gap_create_connection(SCAN_P, SCAN_L, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, CONN_P1, CONN_P2, 0,
                                        SUPERV_TIMEOUT, CONN_L1 , CONN_L2);
        if (ret != 0) {
            printf("Error while starting connection.\n");
            Clock_Wait(100);
        }else{
            host_connect_init_flag = TRUE;
        }
        printf("connection init\n\r");

    }
   
}

void ble_host_discover_char(void* arg)
{
    /* Start TX handle Characteristic dynamic discovery if not yet done */
    if (host_connected && !end_read_write_char_handle) {
        printf("1\n");
        read_write_char_handle();
    }/* Start RX handle Characteristic dynamic discovery if not yet done */
    else if (host_connected && !end_read_notify_read_char_handle) {
        read_notify_read_char_handle();
        printf("2\n");
    }
    else if (host_connected && !end_read_write_without_rsp_char_handle) { 
        read_write_without_rsp_char_handle();
        printf("3\n");
    }
    else if (host_connected && !end_read_notify_char_handle) {
        read_notify_char_handle();
        printf("4\n");
    }

    if(host_connected && end_read_write_char_handle && end_read_notify_read_char_handle && end_read_write_without_rsp_char_handle && end_read_notify_char_handle && !host_notification_enabled) {
        BSP_LED_Off(LED0); //end of the connection and chars discovery phase
        enableNotification();
        printf("discover over\n\r");
    }else{
     printf("host_connected: %x, end_read_write_char_handle : %x\n", host_connected, end_read_write_char_handle);
    }
   
}

/*start read write char handle*/
static void read_write_char_handle(void)
{
    if (!start_read_write_char_handle)
    {
        printf("Start reading write_char_handle\n");

        const uint8_t charUuid128_TX[16] = { 0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x01, 0x80, 0x00, 0x00};
        aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
        start_read_write_char_handle = TRUE;
    }

}

/*start read notify read char handle*/
static void read_notify_read_char_handle(void)
{
    if (!start_read_notify_read_char_handle)
    {
        printf("Start reading notify_read_char_handle\n");

        const uint8_t charUuid128_RX[16] = {0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x02, 0x80, 0x00, 0x00};
        aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_RX);
        start_read_notify_read_char_handle = TRUE;
    }
}
/*start read write without rsp char handle*/
static void read_write_without_rsp_char_handle(void)
{
    if (!start_read_write_without_rsp_char_handle)
    {
        printf("Start reading write_without_rsp_char_handle\n");

        const uint8_t charUuid128_TX[16] = { 0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x03, 0x80, 0x00, 0x00};
        aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
        start_read_write_without_rsp_char_handle = TRUE;
    }

}
/*start read notify char handle*/
static void read_notify_char_handle(void)
{
    if (!start_read_notify_char_handle)
    {
        printf("Start reading notify_char_handle\n");

        const uint8_t charUuid128_TX[16] = { 0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x04, 0x80, 0x00, 0x00};
        aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
        start_read_notify_char_handle = TRUE;
    }

}

/*  Enable notification */
static void enableNotification(void)
{
    uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
    struct timer t;
    Timer_Set(&t, CLOCK_SECOND*10);

    while(aci_gatt_write_charac_descriptor(connection_handle, notify_read_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) {
        /* Radio is busy */
        if(Timer_Expired(&t)) break;
    }
    host_notification_enabled = TRUE;
    HAL_Delay(100);
    ble_host_on_connect();
    printf("notification enable\n\r");
}

/*host send message*/
tBleStatus ble_host_send(uint8_t type, uint32_t length, uint8_t* value)
{
    tBleStatus ret;
    uint8_t packet[20];
    if(host_notification_enabled == FALSE) {

        return BLE_WAIT_ENABLE_NOTIFY;
    }

    if (length > 18) length = 18;
    packet[0] = type;
    packet[1] = length;
    memcpy(packet + 2, value, length);
    ret = aci_gatt_write_charac_value(connection_handle, write_handle+1, packet[1]+2, packet);
    if (ret != BLE_STATUS_SUCCESS) {

        return BLE_STATUS_ERROR ;
    }
    
    return ret;
}








