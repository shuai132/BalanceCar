
#include "hal.h"
#include "hal_types.h"
#include "ble_status.h"
#include "bluenrg_aci.h"
#include "bluenrg_utils.h"
#include "hci.h"
#include "osal.h"
#include "string.h"
#include "stm32_bluenrg_ble.h"

#define SUPPORTED_BOOTLOADER_VERSION_MIN  3
#define SUPPORTED_BOOTLOADER_VERSION_MAX  5

#define BASE_ADDRESS 0x10010000

#define FW_OFFSET       (2*1024)  // 2 KB
#define FULL_STACK_SIZE (66*1024) // 66 KB
#define BOOTLOADER_SIZE (2*1024)  // 2 kB
#define SECTOR_SIZE     (2*1024)  // 2 KB
#define DATA_SIZE       64        // 64 bytes

// x**32 + x**26 + x**23 + x ** 22 + x**16 + x**12 + x**11 +
// x**10 + x**8 + x**7 + x**5 + x**4 + x**2 + x**1 + x**0
#define CRC_POLY        0x04C11DB7      // the poly without the x**32

#define BOOTLOADER_CRC_NOT_PATCHED 0x878FB3FC

#define IFR_SIZE 192
#define IFR_BASE_ADDRESS 0x10020000
#define IFR_CONFIG_DATA_OFFSET (SECTOR_SIZE-IFR_SIZE)  // Offset in IFR sector containing configuration data

#if BLUENRG_MS
#define IFR_WRITE_OFFSET_BEGIN IFR_CONFIG_DATA_OFFSET
#else
#define IFR_WRITE_OFFSET_BEGIN 0
#endif


#define BLUE_FLAG_OFFSET 0x8C0
#define MAX_ERASE_RETRIES 2
#define MAX_WRITE_RETRIES 2
#define MIN_WRITE_BLOCK_SIZE 4

#define RETRY_COMMAND(func, num_ret, error)  \
{					\
  uint8_t num_retries;                  \
  num_retries = 0;			\
  error = 0;				\
  while (num_retries++ < num_ret)  {	\
    if (func == BLE_STATUS_SUCCESS)	\
      break;				\
    if (num_retries == num_ret)		\
      error = BLE_UTIL_ACI_ERROR;	\
  }					\
}

typedef struct{
  uint8_t cold_ana_act_config_table[64];
}cold_table_TypeDef;

/* This function calculates the CRC of a sector of flash, if bytes passed are less than sector size, 
   they are extended with 0xFF until sector size is reached
*/
static uint32_t updater_calc_crc(const uint8_t* data, uint16_t nr_of_bytes)
{
    uint32_t i, j, a1;
    uint32_t crc, value;

    crc = 0;
    for (i = 0; i < SECTOR_SIZE; i += 4) {
      uint8_t *dataw = (uint8_t *) &value;
     
      dataw[0] = (i < nr_of_bytes) ? data[i] : 0xFF;
      dataw[1] = ((i + 1) < nr_of_bytes) ? data[i+1] : 0xFF;
      dataw[2] = ((i + 2) < nr_of_bytes) ? data[i+2] : 0xFF;
      dataw[3] = ((i + 3) < nr_of_bytes) ? data[i+3] : 0xFF;

      crc = crc ^ value;
      for (j = 0; j < 32; j ++) {
	a1 = (crc >> 31) & 0x1;
	crc = (crc << 1) ^ (a1 * CRC_POLY);
      }
    }

    return crc;
}

int program_device(const uint8_t *fw_image, uint32_t fw_size)
{
  uint8_t version, num_erase_retries=0, status, data_size;
  uint8_t number_sectors, module;
  uint32_t address, j;
  uint32_t crc, crc2, crc_size;
  
  BlueNRG_HW_Bootloader();
  HCI_Process(); // To receive the EVT_INITIALIZED

  if(aci_get_updater_version(&version))
    return BLE_UTIL_ACI_ERROR;
  
  if(version < SUPPORTED_BOOTLOADER_VERSION_MIN || version > SUPPORTED_BOOTLOADER_VERSION_MAX)
    return BLE_UTIL_UNSUPPORTED_VERSION;
  
  if (fw_size != FULL_STACK_SIZE)
    return BLE_UTIL_WRONG_IMAGE_SIZE;

  if (fw_size % MIN_WRITE_BLOCK_SIZE)
    return BLE_UTIL_WRONG_IMAGE_SIZE;

  /* Calculate the number of sectors necessary to contain the fw image.*/
  number_sectors = ((fw_size + SECTOR_SIZE - 1) / SECTOR_SIZE);

  /***********************************************************************
  * Erase BLUE flag
  ************************************************************************/
  RETRY_COMMAND(aci_erase_blue_flag(), MAX_WRITE_RETRIES, status);
  if (status != BLE_STATUS_SUCCESS)
    return status;

  /***********************************************************************
  * Erase and Program sectors
  ************************************************************************/  
  for(int i = FW_OFFSET; i < (number_sectors * SECTOR_SIZE); i += SECTOR_SIZE) {
    num_erase_retries = 0;
    while (num_erase_retries++ < MAX_ERASE_RETRIES) {
      aci_updater_erase_sector(BASE_ADDRESS + i);
      if ((i/SECTOR_SIZE) < (number_sectors-1))
	data_size = DATA_SIZE;
      else
	data_size = MIN_WRITE_BLOCK_SIZE;	
      for (j=i; ((j<SECTOR_SIZE+i)&&(j<fw_size)); j += data_size) {
	RETRY_COMMAND(aci_updater_program_data_block(BASE_ADDRESS+j, data_size, fw_image+j), MAX_WRITE_RETRIES, status);
	if (status != BLE_STATUS_SUCCESS)
	  break;
      }
      if (status == BLE_STATUS_SUCCESS)
	break;
    }
    if (num_erase_retries == MAX_ERASE_RETRIES)
      return BLE_UTIL_ACI_ERROR;
  }
  
  /***********************************************************************
  * Verify firmware
  ************************************************************************/
  module = fw_size % SECTOR_SIZE;
  crc_size = SECTOR_SIZE;
  for(int i = SECTOR_SIZE; i < (number_sectors*SECTOR_SIZE); i += SECTOR_SIZE){
    address = BASE_ADDRESS + i;
    if(aci_updater_calc_crc(address, 1, &crc))
      return BLE_UTIL_ACI_ERROR;
    
    if ((module != 0)  && ((i/SECTOR_SIZE) == (number_sectors-1)))
	crc_size = module;

    crc2 = updater_calc_crc(fw_image+i,crc_size);
    if(crc!=crc2)
      return BLE_UTIL_CRC_ERROR;
  }

  /***********************************************************************
  * Write BLUE flag
  ************************************************************************/
  RETRY_COMMAND(aci_reset_blue_flag(), MAX_WRITE_RETRIES, status);
  if (status != BLE_STATUS_SUCCESS)
    return status;
  
  BlueNRG_RST();
  HCI_Process(); // To receive the EVT_INITIALIZED
  
  return BLE_STATUS_SUCCESS;
}

int read_IFR(uint8_t *data)
{
  uint8_t version, offset;
  tBleStatus ret;
  
  offset = 0;
  aci_updater_start();
  if(aci_get_updater_version(&version))
    return BLE_UTIL_ACI_ERROR;
  
  if(version < SUPPORTED_BOOTLOADER_VERSION_MIN || version > SUPPORTED_BOOTLOADER_VERSION_MAX)
    return BLE_UTIL_UNSUPPORTED_VERSION;
  
  /***********************************************************************
  * Reading last 3 IFR 64-byte blocks
  ************************************************************************/
  for(int i = (FULL_STACK_SIZE - IFR_SIZE); i < FULL_STACK_SIZE; i += DATA_SIZE){
    ret = aci_updater_read_data_block(BASE_ADDRESS+i, DATA_SIZE, (data+offset));
    offset += DATA_SIZE;
    if(ret) return BLE_UTIL_ACI_ERROR;
  }
  
  BlueNRG_RST();
  HCI_Process(); // To receive the EVT_INITIALIZED

  return BLE_STATUS_SUCCESS;
  
}

void parse_IFR_data_config(const uint8_t data[64], IFR_config2_TypeDef *IFR_config)
{
  IFR_config->stack_mode = data[0];
  IFR_config->slave_sca_ppm = LE_TO_HOST_16(data+28);
  IFR_config->master_sca = data[30];
  IFR_config->hs_startup_time = LE_TO_HOST_16(data+32);
  IFR_config->year = BCD_TO_INT(data[41]);
  IFR_config->month = BCD_TO_INT(data[42]);
  IFR_config->day = BCD_TO_INT(data[43]);    
}

int IFR_validate(IFR_config2_TypeDef *IFR_config)
{
  if(IFR_config->stack_mode < 1 || IFR_config->stack_mode > 3)
    return BLE_UTIL_PARSE_ERROR; // Unknown Stack Mode
  if(IFR_config->master_sca > 7)
    return BLE_UTIL_PARSE_ERROR; // Invalid Master SCA
  if(IFR_config->month > 12 || IFR_config->month < 1)
    return BLE_UTIL_PARSE_ERROR; // Invalid date
  if(IFR_config->day > 31 || IFR_config->day < 1)
    return BLE_UTIL_PARSE_ERROR; // Invalid date
  if(IFR_config->month > 12 || IFR_config->month < 1)
    return BLE_UTIL_PARSE_ERROR; // Invalid date
  
  return BLE_STATUS_SUCCESS;
}

/* TODO: Function to generate data from given options. */

void change_IFR_data_config(IFR_config2_TypeDef *IFR_config, uint8_t data[64])
{
  data[0] = IFR_config->stack_mode;
  HOST_TO_LE_16(data+28, IFR_config->slave_sca_ppm);
  data[30] = IFR_config->master_sca;
  HOST_TO_LE_16(data+32, IFR_config->hs_startup_time);  
  data[41] = INT_TO_BCD(IFR_config->year);
  data[42] = INT_TO_BCD(IFR_config->month);
  data[43] = INT_TO_BCD(IFR_config->day);
}


int program_IFR(const IFR_config_TypeDef *ifr_image)
{
  uint8_t version, num_erase_retries;
  tBleStatus ret;
#if BLUENRG_MS
  const uint8_t *ifr_data = (uint8_t *)ifr_image;
#else
  uint8_t ifr_data[SECTOR_SIZE];
#endif
  uint8_t hwVersion;
  uint16_t fwVersion;
    
  if(getBlueNRGVersion(&hwVersion, &fwVersion))
    return BLE_UTIL_ACI_ERROR;
  
  BlueNRG_HW_Bootloader();
  HCI_Process(); // To receive the EVT_INITIALIZED
  
  if(aci_get_updater_version(&version))
    return BLE_UTIL_ACI_ERROR;
  
  if(version < SUPPORTED_BOOTLOADER_VERSION_MIN || version > SUPPORTED_BOOTLOADER_VERSION_MAX)
    return BLE_UTIL_UNSUPPORTED_VERSION;
  
#ifndef BLUENRG_MS
    /***********************************************************************
   * READ IFR data
   ************************************************************************/  
  for(int i = 0; i < SECTOR_SIZE; i += DATA_SIZE){
    ret = aci_updater_read_data_block(IFR_BASE_ADDRESS+i, DATA_SIZE, ifr_data+i);
    if(ret != BLE_STATUS_SUCCESS){
      return ret;
    }
  }
#endif
  
  /***********************************************************************
  * Erase & Flashing IFR sectors
  ************************************************************************/
#ifndef BLUENRG_MS  
  Osal_MemCpy(&ifr_data[SECTOR_SIZE-IFR_SIZE], ifr_image, IFR_SIZE);
#endif
  num_erase_retries = 0;
  while (num_erase_retries++ < MAX_ERASE_RETRIES) {
    aci_updater_erase_sector(IFR_BASE_ADDRESS);
    for(int i = IFR_WRITE_OFFSET_BEGIN, j = 0; i < SECTOR_SIZE; i += DATA_SIZE, j += DATA_SIZE) {
      RETRY_COMMAND(aci_updater_program_data_block(IFR_BASE_ADDRESS+i, DATA_SIZE, ifr_data+j), MAX_WRITE_RETRIES, ret);
      if (ret != BLE_STATUS_SUCCESS)
	break;
    }
    if (ret == BLE_STATUS_SUCCESS)
      break;
  }
  if (num_erase_retries == MAX_ERASE_RETRIES)
    return BLE_UTIL_ACI_ERROR;

  /***********************************************************************
  * Verify IFR
  ************************************************************************/  
  {
    uint8_t ifr_updated[DATA_SIZE];
    for(int i = IFR_WRITE_OFFSET_BEGIN, j = 0; i < SECTOR_SIZE; i += DATA_SIZE, j += DATA_SIZE){
      ret = aci_updater_read_data_block(IFR_BASE_ADDRESS+i, DATA_SIZE, ifr_updated);
      if(ret != BLE_STATUS_SUCCESS){
	return ret;
      }
      if (memcmp(ifr_updated, ifr_data+j, DATA_SIZE) != 0)
	return BLE_UTIL_WRONG_VERIFY;
    }
  }

  BlueNRG_RST();
  HCI_Process(); // To receive the EVT_INITIALIZED
    
  return BLE_STATUS_SUCCESS;
}

uint8_t verify_IFR(const IFR_config_TypeDef *ifr_data)
{
  uint8_t ifr_updated[DATA_SIZE];
  uint8_t version, ret = BLE_STATUS_SUCCESS;
    
  aci_updater_start();
  if(aci_get_updater_version(&version))
    return BLE_UTIL_ACI_ERROR;
  for(int i = 0; i < IFR_SIZE; i += DATA_SIZE){
    ret = aci_updater_read_data_block((IFR_BASE_ADDRESS+SECTOR_SIZE-IFR_SIZE)+i, DATA_SIZE, ifr_updated);
    if(ret != BLE_STATUS_SUCCESS){
      return ret;
    }
    if (memcmp(ifr_updated, ((uint8_t*)ifr_data)+i, DATA_SIZE) != 0)
    {
      ret = BLE_UTIL_WRONG_VERIFY;
      break;
    }
  }

  BlueNRG_RST();
  HCI_Process(); // To receive the EVT_INITIALIZED
  
  return ret;
}

uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_le_read_local_version(&hci_version, &hci_revision, &lmp_pal_version, 
				     &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
    *fwVersion = (hci_revision & 0xFF) << 8;              // Major Version Number
    *fwVersion |= ((lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    *fwVersion |= lmp_pal_subversion & 0xF;               // Patch Version Number
  }

  HCI_Process(); // To receive the BlueNRG EVT

  return status;
}

uint8_t getBlueNRGUpdaterVersion(uint8_t *version)
{

  BlueNRG_HW_Bootloader();
  HCI_Process(); // To receive the EVT_INITIALIZED

  if(aci_get_updater_version(version))
    return BLE_UTIL_ACI_ERROR;
  
  if(*version < SUPPORTED_BOOTLOADER_VERSION_MIN || *version > SUPPORTED_BOOTLOADER_VERSION_MAX)
    return BLE_UTIL_UNSUPPORTED_VERSION;

  BlueNRG_RST();
  HCI_Process(); // To receive the EVT_INITIALIZED

  return BLE_STATUS_SUCCESS;
}

uint8_t isHWBootloader_Patched(void)
{
  uint8_t status, version;
  uint32_t crc, address = 0x10010000;

  if(aci_get_updater_version(&version))
    return BLE_UTIL_ACI_ERROR;

  RETRY_COMMAND(aci_updater_calc_crc(address, 1, &crc), 2, status);
  if (status != BLE_STATUS_SUCCESS)
    return 0;

  if (crc == BOOTLOADER_CRC_NOT_PATCHED)
    return 0;

  return 1;
}
