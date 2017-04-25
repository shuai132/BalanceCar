
#include "bluenrg_sdk_api.h"
#include "bluenrg_sdk_host_api.h"

#if NO_PRINTF
#define printf(...)
#endif

#define BDADDR_SIZE 6

#ifdef CLIENT_ROLE
extern uint8_t host_connect_init_flag;
extern volatile int host_connected;
extern volatile uint8_t host_notification_enabled;
extern volatile uint8_t start_read_write_char_handle;
extern volatile uint8_t start_read_notify_read_char_handle;
extern volatile uint8_t start_read_write_without_rsp_char_handle;
extern volatile uint8_t start_read_notify_char_handle;
extern volatile uint8_t end_read_write_char_handle;
extern volatile uint8_t end_read_notify_read_char_handle;
extern volatile uint8_t end_read_write_without_rsp_char_handle;
extern volatile uint8_t end_read_notify_char_handle;

extern uint16_t write_handle;
extern uint16_t notify_read_handle;
extern uint16_t write_without_rsp_handle;
extern uint16_t notify_handle;

#endif

/*adv parameter structure*/
typedef struct
{
    uint8_t               type;                 /**< See @ref BLE_GAP_ADV_TYPES. */
    uint8_t               fp;                   /**< Filter Policy, see @ref BLE_GAP_ADV_FILTER_POLICIES. */
    uint16_t              interval;             /**< Advertising interval between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s), see @ref BLE_GAP_ADV_INTERVALS.
                                                   - If type equals @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND, this parameter must be set to 0 for high duty cycle directed advertising.
                                                   - If type equals @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND, set @ref BLE_GAP_ADV_INTERVAL_MIN <= interval <= @ref BLE_GAP_ADV_INTERVAL_MAX for low duty cycle advertising.*/
    uint16_t              timeout;              /**< Advertising timeout between 0x0001 and 0x3FFF in seconds, 0x0000 disables timeout. See also @ref BLE_GAP_ADV_TIMEOUT_VALUES. If type equals @ref BLE_GAP_ADV_TYPE_ADV_DIRECT_IND, this parameter must be set to 0 for High duty cycle directed advertising. */

} ble_gap_adv_params_t;

ble_gap_adv_params_t m_adv_params;
static uint8_t adv_name[20] = "BlueNRG", adv_name_len = 20,  local_name[20], local_name_len;

uint16_t connection_handle = 0 ,notification_enabled = 0;

volatile uint8_t Ble_conn_state = BLE_CONNECTABLE;
uint16_t BLueNrgServHandle =0x0001, WriteCharHandle = 0x0006, WriteCmdCharHandle = 0x000A, ReadNotifyCharHandle = 0x000D, NotifyCharHandle = 0x0010;
uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

__weak void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value) {}
__weak void ble_device_on_connect( void ) {}
__weak void ble_device_on_disconnect(uint8_t reason) {}


/*Record Connection Handle*/
static void connection_information(uint16_t handle);
/*Read Req */
static void Read_Request_CB(uint16_t handle);
/*Add Service*/
static tBleStatus Add_Service(void);
/*init gap/gatt service*/
/*Init BLUENrg, HCI, Add Service*/
tBleStatus ble_init_bluenrg(void);

#ifdef CLIENT_ROLE
BLE_RoleTypeDef BLE_Role = CLIENT;
#else
BLE_RoleTypeDef BLE_Role = SERVER;
#endif


/**
	*@brief  Board Initializition
	*@param  None
	*@retval ret
	*/
tBleStatus ble_init_bluenrg(void)
{
    tBleStatus ret;
    /*gatt_Init*/
    ret = aci_gatt_init();

    if(ret) {
        return BLE_GATT_INIT_FAILED;
    }

    if(BLE_Role == SERVER) {
#ifdef BLUENRG_MS
        /*BUG: Name Length*/
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, adv_name_len, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#else
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#endif
    } else {
#ifdef BLUENRG_MS
        /*BUG: Name Length*/
        ret = aci_gap_init(GAP_CENTRAL_ROLE, 0, adv_name_len, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#else
        ret = aci_gap_init(GAP_CENTRAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#endif
    }  
    if(ret != BLE_STATUS_SUCCESS) {
        printf("GAP_Init failed.\n");
    }

    if(ret != BLE_STATUS_SUCCESS) {
        return BLE_GAP_INIT_FAILED;
    }

    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    if (ret == BLE_STATUS_SUCCESS) {
        printf("BLE Stack Initialized.\n");
    }

    if (ret != BLE_STATUS_SUCCESS) {
        return BLE_STACK_INIT_FAILED;
    }
    if(BLE_Role == SERVER) {
        printf("SERVER: BLE Stack Initialized\n");
        /* add JUMA SERVICE*/
        ret = Add_Service();
        if(ret == BLE_STATUS_SUCCESS)
            printf("Service added successfully.\n");
        if(ret != BLE_STATUS_SUCCESS) {
            return BLE_ADD_SERVICE_FAILED;
        }
    } else {
        printf("CLIENT: BLE Stack Initialized\n");
    }

    return 0;
}

/**
	*@brief Set Tx Power level
	*@param Level
	*@retval ret
*/
tBleStatus ble_set_tx_power(uint8_t level)
{
    tBleStatus ret;
    /* Set output power level */
    ret = aci_hal_set_tx_power_level(1,level);

    return ret;
}
/**
	*@brief  Add Service
	*@param  None
	*@retval ret
	*/
static tBleStatus Add_Service(void)
{
    tBleStatus ret;
    uint8_t service_uuid[16] = { 0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x00, 0x80, 0x00, 0x00};
    uint8_t command_uuid[16] = { 0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x01, 0x80, 0x00, 0x00};
    uint8_t event_char_uuid[16] = {  0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x02, 0x80, 0x00, 0x00};
    uint8_t bulkout_uuid[16] = { 0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x03, 0x80, 0x00, 0x00};
    uint8_t bulkin_uuid[16] = {  0x8C, 0xF9, 0x97,0xA6, 0xEE, 0x94, 0xE3,0xBC,0xF8, 0x21, 0xB2, 0x60, 0x04, 0x80, 0x00, 0x00};
    /*add service*/
    ret = aci_gatt_add_serv(UUID_TYPE_128,  service_uuid, PRIMARY_SERVICE, 11,
                            &BLueNrgServHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

    /*Add command characteristic*/
    ret =  aci_gatt_add_char(BLueNrgServHandle, UUID_TYPE_128, command_uuid, 0x14,
                             CHAR_PROP_WRITE, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                             16, 1, &WriteCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

    /*Add bulkout characteristic*/
    ret =  aci_gatt_add_char(BLueNrgServHandle, UUID_TYPE_128, bulkout_uuid, 0x14,
                             CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, 0,
                             16, 1, &WriteCmdCharHandle);//GATT_NOTIFY_ATTRIBUTE_WRITE
    if (ret != BLE_STATUS_SUCCESS) goto fail;

    /*Add event characteristic*/
    ret =  aci_gatt_add_char(BLueNrgServHandle, UUID_TYPE_128, event_char_uuid, 0x14,
                             CHAR_PROP_NOTIFY|CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                             16, 1, &ReadNotifyCharHandle);//GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP
    if (ret != BLE_STATUS_SUCCESS) goto fail;



    /*Add bulkin characteristic*/
    ret =  aci_gatt_add_char(BLueNrgServHandle, UUID_TYPE_128, bulkin_uuid, 0x14,
                             CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                             16, 1, &NotifyCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

    return BLE_STATUS_SUCCESS;

fail:
    return BLE_STATUS_ERROR ;
}

/**
	*@brief  Set Adv Local Name
	*@param  None
	*@retval ret
	*/
tBleStatus ble_device_set_name(const char* new_device_name)
{
    adv_name_len = strlen(new_device_name);
    local_name_len = adv_name_len+1;
    memcpy(adv_name,new_device_name,adv_name_len);
    local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(local_name+1,new_device_name,adv_name_len);

    return BLE_STATUS_SUCCESS;
}

/**
	*@brief Adv Address
	*@param Adv Address
	*@retval ret
	*/
tBleStatus ble_address(uint8_t* advaddress)
{
    tBleStatus ret;
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                    CONFIG_DATA_PUBADDR_LEN,
                                    advaddress);
    if(ret) {
        return BLE_SET_BD_ADDR_FAILED;
    }

    return 0;
}

/**
	*@brief	 Config adv param and ready to adv
	*@param	 Advname,AdvAddress,TxPowerLevel,Advinterval
	*@retval None
	*/
void ble_set_adv_param(char* adv_name, uint8_t*adv_address, uint8_t tx_power_pevel, uint16_t adv_interval)
{
    /*Set Adv Address*/
    ble_address(adv_address);
    /*Set Adv Name*/
    ble_device_set_name(adv_name);
    /*Set Tx Power Level*/
    ble_set_tx_power(tx_power_pevel);
    /* Range: 0x0020 to 0x4000
    	 Default: 1.28 s
    	 Time = AdvInterval * 0.625 msec
    */
    ble_device_set_advertising_interval(adv_interval);
}


/**
	*@brief 	Start To Adv
	*@param  	None
	*@retval 	ret
	*/
tBleStatus ble_device_start_advertising(void)
{
    tBleStatus ret;
    uint8_t uuid_length = 3;
    uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST,0x90,0xFE};

    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    HAL_Delay(1);
    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                     adv_name_len, adv_name);
      if(ret){
         printf("aci_gatt_update_char_value failed.\n");
      }
    /*min_adv_interval > 32*0.625*/
    ret = aci_gap_set_discoverable(ADV_IND, m_adv_params.interval, m_adv_params.interval, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                  local_name_len, (char*)local_name, uuid_length, serviceUUIDList, 0, 0);//// start advertising
    
    return ret;
}

/**
	*@brief 	Start Adv
	*@param  	None
	*@retval 	ret
	*/
tBleStatus ble_device_send(uint8_t type, uint32_t length, uint8_t* value)
{
    tBleStatus ret;
    uint8_t packet[20];
    if(notification_enabled == 0) {

        return BLE_WAIT_REMOTE_ENABLE_NOTIFY;
    }

    if (length > 18) length = 18;
    packet[0] = type;
    packet[1] = length;
    memcpy(packet + 2, value, length);
    ret = aci_gatt_update_char_value(BLueNrgServHandle, ReadNotifyCharHandle, 0, packet[1]+2, packet);
    if (ret != BLE_STATUS_SUCCESS) {

        return BLE_STATUS_ERROR ;
    }

    return 0;
}
/**
	*@brief 	Stop Adv
	*@param  	None
	*@retval 	ret
	*/
tBleStatus ble_device_stop_advertising(void)
{
    tBleStatus ret;
    ret = aci_gap_set_non_discoverable();

    return ret;
}
/**
	*@brief 	Config Adv Interval (min_adv_interval > 32*0.625)
	*@param  	None
	*@retval 	ret
	*/
void ble_device_set_advertising_interval(uint16_t interval)
{
    /*min_adv_interval > 32*0.625*/
    m_adv_params.interval = interval;
}
/**
	*@brief 	To disconnect Ble Connection
	*@param  	None
	*@retval 	ret
	*/
tBleStatus ble_user_disconnect_device(void)
{
    tBleStatus ret;
    ret = aci_gap_terminate(connection_handle, HCI_OE_USER_ENDED_CONNECTION);
    Ble_conn_state = BLE_CONNECTABLE;
    return ret;
}
/**
	*@brief 	Record Connection Handle
	*@param  	None
	*@retval 	None
	*/
static void connection_information(uint16_t handle)
{
    connection_handle = handle;

}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
 #ifdef CLIENT_ROLE
    host_connected = FALSE;
    host_connect_init_flag = FALSE;
    printf("Disconnected\n");
    /* Make the device connectable again. */
    host_notification_enabled = FALSE;
    start_read_write_char_handle = FALSE;
    start_read_write_without_rsp_char_handle = FALSE;
    start_read_notify_read_char_handle = FALSE;
    start_read_notify_char_handle = FALSE;
    end_read_write_char_handle = FALSE;
    end_read_write_without_rsp_char_handle = FALSE;
    end_read_notify_read_char_handle = FALSE;
    end_read_notify_char_handle = FALSE;
 #endif
}

/*connection complete callback*/
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
 #ifdef CLIENT_ROLE
    host_connected = TRUE;
    connection_handle = handle;

    printf("Connected to device:\n\r");
    for(int i = 5; i > 0; i--) {
        printf("%02X-", addr[i]);
    }
    printf("%02X\n", addr[0]);
    
    /*discover device*/
    ble_host_discover_char(NULL);
 #endif

}


/*gatt notification call back*/
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
  #ifdef CLIENT_ROLE
    if (BLE_Role == CLIENT) {
        if(attr_handle == (notify_read_handle+1)) {
            ble_host_on_message(attr_value[0], attr_value[1], attr_value+2);
        }

    } else {

    }
  #endif
}


/**
	*@brief 	Read Req
	*@param  	None
	*@retval 	None
	*/
static void Read_Request_CB(uint16_t handle)
{
    if(handle == BLueNrgServHandle + 1) {
        //Acc_Update((AxesRaw_t*)&axes_data);
    }

    if(connection_handle != 0) {
        aci_gatt_allow_read(connection_handle);
    }
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
    hci_uart_pckt *hci_pckt = pckt;
    /* obtain event packet */
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
    if(hci_pckt->type != HCI_EVENT_PKT)
        return;

    switch(event_pckt->evt) {

    case EVT_DISCONN_COMPLETE:
    {
        #ifdef CLIENT_ROLE
          host_notification_enabled = 0;
        #endif
        notification_enabled = 0;
        ble_device_on_disconnect(event_pckt->data[3]);
         /*Host*/
         GAP_DisconnectionComplete_CB();
    }
    break;

    case EVT_LE_META_EVENT:
    {
        evt_le_meta_event *evt = (void *)event_pckt->data;
        switch(evt->subevent) {
        case EVT_LE_CONN_COMPLETE:
        {

            ble_device_on_connect();
            evt_le_connection_complete *cc = (void *)evt->data;
            connection_information(cc->handle);
            /*host*/
            GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);

        }
        break;
        case EVT_LE_ADVERTISING_REPORT:
        {
             #ifdef CLIENT_ROLE
              le_advertising_info* adv_data = (void *)((event_pckt->data)+2);
           
              ble_host_device_found(adv_data);
            #endif
        }
        break;
        }
    }
    break;

    case EVT_VENDOR:
    {
        evt_blue_aci *blue_evt = (void*)event_pckt->data;
        switch(blue_evt->ecode) {
        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
            /* this callback is invoked when a GATT attribute is modified
            extract callback data and pass to suitable handler function */
            evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data;
            ///on message
            if(evt->att_data[1] > 0) {
                ble_device_on_message(evt->att_data[0], evt->att_data[1], (evt->att_data)+2);
            } else if(evt->att_data[0] == 1) {
                notification_enabled = 1;
                #ifdef CLIENT_ROLE
                  host_notification_enabled = 1;
                #endif
            }
        }
        break;
        case EVT_BLUE_GATT_NOTIFICATION:
        {
            evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
            GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
        }
        break;
        case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
            evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
            Read_Request_CB(pr->attr_handle);
        }
        break;
        case EVT_BLUE_GAP_DEVICE_FOUND:
        {
            printf("scanned one device\n\r");
        }
        break;
        case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
        {

        }
        break;
#if 1
        case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
            #ifdef CLIENT_ROLE
              if(BLE_Role == CLIENT) {
                  printf("EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP\n");

                  evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;

                  if (start_read_write_char_handle && !end_read_write_char_handle)
                  {
                      write_handle = resp->attr_handle;
                      printf("write_handle  %04X\n", write_handle);
                  }
                  else if (start_read_notify_read_char_handle && !end_read_notify_read_char_handle)
                  {
                      notify_read_handle = resp->attr_handle;
                      printf("notify_read_handle  %04X\n", notify_read_handle);
                  }
                  else if (start_read_write_without_rsp_char_handle && !end_read_write_without_rsp_char_handle)
                  {
                      write_without_rsp_handle = resp->attr_handle;
                      printf("write_without_rsp_handle  %04X\n", write_without_rsp_handle);
                  }
                  else if (start_read_notify_char_handle && !end_read_notify_char_handle)
                  {
                      notify_handle = resp->attr_handle;
                      printf("notify_handle %04X\n", notify_handle);
                  }
              }
            #endif
            break;

        case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
            #ifdef CLIENT_ROLE
              if(BLE_Role == CLIENT) {
                  /* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
                  //evt_gatt_procedure_complete *pr = (void*)blue_evt->data;

                  if (start_read_write_char_handle && !end_read_write_char_handle)
                  {
                      end_read_write_char_handle = TRUE;
                      run_when_idle(ble_host_discover_char, NULL);

                  }
                  else if (start_read_notify_read_char_handle && !end_read_notify_read_char_handle)
                  {
                      end_read_notify_read_char_handle = TRUE;
                      run_when_idle(ble_host_discover_char, NULL);
                  }
                  else if (start_read_write_without_rsp_char_handle && !end_read_write_without_rsp_char_handle)
                  {
                      end_read_write_without_rsp_char_handle = TRUE;
                      run_when_idle(ble_host_discover_char, NULL);
                  }
                  else if (start_read_notify_char_handle && !end_read_notify_char_handle)
                  {
                      end_read_notify_char_handle = TRUE;
                      run_when_idle(ble_host_discover_char, NULL);
                  }
              }
            #endif
            break;
#endif
        }
    }
    break;
    }
}
