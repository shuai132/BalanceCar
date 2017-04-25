/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : bluenrg_hal_aci.h
* Author             : AMS - AAS
* Version            : V1.0.0
* Date               : 26-Jun-2014
* Description        : Header file with HCI commands for BlueNRG FW6.3.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __BLUENRG_HAL_ACI_H__
#define __BLUENRG_HAL_ACI_H__

/**
 *@addtogroup HAL HAL
 *@brief Hardware Abstraction Layer.
 *@{
 */

/**
 * @defgroup HAL_Functions HAL functions
 * @brief API for BlueNRG HAL layer.
 * @{
 */

/**
 * @brief This command writes a value to a low level configure data structure.
 * @note  It is useful to setup directly some low level parameters for the system at runtime.
 * @param offset Offset in the data structure. The starting member in the data structure will have an offset 0.\n
 * 				 See @ref Config_vals.
 *
 * @param len Length of data to be written
 * @param[out] val Data to be written
 * @return Value indicating success or error code.
 */
tBleStatus aci_hal_write_config_data(uint8_t offset, 
                                    uint8_t len,
                                    const uint8_t *val);

/**
 * @brief This command sets the TX power level of the BlueNRG.
 * @note  By controlling the EN_HIGH_POWER and the PA_LEVEL, the combination of the 2 determines
 *        the output power level (dBm).
 *        When the system starts up or reboots, the default TX power level will be used, which is
 *        the maximum value of 8dBm. Once this command is given, the output power will be changed
 *        instantly, regardless if there is Bluetooth communication going on or not. For example,
 *        for debugging purpose, the BlueNRG can be set to advertise all the time and use this
 *        command to observe the signal strength changing. The system will keep the last received
 *        TX power level from the command, i.e. the 2nd command overwrites the previous TX power
 *        level. The new TX power level remains until another Set TX Power command, or the system
 *        reboots.\n
 * @param en_high_power Can be only 0 or 1. Set high power bit on or off. It is strongly adviced to use the
 * 						right value, depending on the selected hardware configuration for the RF network:
 * 						normal mode or high power mode.
 * @param pa_level Can be from 0 to 7. Set the PA level value.
 * @return Value indicating success or error code.
 */
tBleStatus aci_hal_set_tx_power_level(uint8_t en_high_power, uint8_t pa_level);

/**
 * @brief Put the device in standby mode.
 * @note Normally the BlueNRG will automatically enter sleep mode to save power. This command puts the
 * 		 device into the Standby mode instead of the sleep mode. The difference is that, in sleep mode,
 * 		 the device can still wake up itself with the internal timer. But in standby mode, this timer is
 * 		 disabled. So the only possibility to wake up the device is by external signals, e.g. a HCI command
 * 		 sent via SPI bus.
 * 		 The command is only accepted when there is no other Bluetooth activity. Otherwise an error code
 * 		 ERR_COMMAND_DISALLOWED will be returned.
 *
 * @return Value indicating success or error code.
 */
tBleStatus aci_hal_device_standby(void);

/**
 * @brief This command starts a carrier frequency, i.e. a tone, on a specific channel.
 * @note  The frequency sine wave at the specific channel may be used for test purpose only.
 * 		  The channel ID is a parameter from 0 to 39 for the 40 BLE channels, e.g. 0 for 2.402GHz, 1 for 2.404GHz etc.
 * 		  This command shouldn't be used when normal Bluetooth activities are ongoing.
 * 		  The tone should be stopped by aci_hal_tone_stop() command.
 *
 * @param rf_channel BLE Channel ID, from 0 to 39 meaning (2.402 + 2*N) GHz. Actually the tone will be emitted at the
 * 					 channel central frequency minus 250 kHz.
 * @return Value indicating success or error code.
 */
tBleStatus aci_hal_tone_start(uint8_t rf_channel);

/**
 * This command is used to stop the previously started aci_hal_tone_start() command.
 * @return Value indicating success or error code.
 */
tBleStatus aci_hal_tone_stop(void);

/**
 * @}
 */

/**
 * @defgroup Config_vals Offsets and lengths for configuration values.
 * @brief Offsets and lengths for configuration values.
 * 		  See aci_hal_write_config_data().
 * @{
 */

/**
 * @name Configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
#define CONFIG_DATA_PUBADDR_OFFSET          (0x00) /**< Bluetooth public address */
#define CONFIG_DATA_DIV_OFFSET              (0x06) /**< DIV used to derive CSRK */
#define CONFIG_DATA_ER_OFFSET               (0x08) /**< Encryption root key used to derive LTK and CSRK */
#define CONFIG_DATA_IR_OFFSET               (0x18) /**< Identity root key used to derive LTK and CSRK */
#define CONFIG_DATA_LL_WITHOUT_HOST         (0x2C) /**< Switch on/off Link Layer only mode. Set to 1 to disable Host.
 	 	 	 	 	 	 	 	 	 	 	 	 	 It can be written only if aci_hal_write_config_data() is the first command
 	 	 	 	 	 	 	 	 	 	 	 	 	 after reset. */

/**
 * Select the BlueNRG roles and mode configurations.\n
 * @li Mode 1: slave or master, 1 connection, RAM1 only (small GATT DB)
 * @li Mode 2: slave or master, 1 connection, RAM1 and RAM2 (large GATT DB)
 * @li Mode 3: master only, 8 connections, RAM1 and RAM2.
 */
#define CONFIG_DATA_ROLE					(0x2D)
/**
 * @}
 */

/**
 * @name Length for configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
#define CONFIG_DATA_PUBADDR_LEN             (6)
#define CONFIG_DATA_DIV_LEN                 (2)
#define CONFIG_DATA_ER_LEN                  (16)
#define CONFIG_DATA_IR_LEN                  (16)
#define CONFIG_DATA_LL_WITHOUT_HOST_LEN     (1)
#define CONFIG_DATA_ROLE_LEN                (1)
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


#endif /* __BLUENRG_HAL_ACI_H__ */
