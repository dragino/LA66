/**
  ******************************************************************************
  * @file    lora_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA_APP_H__
#define __LORA_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>	
#include "Commissioning.h"
#include "LoRaMac.h"
#include "region/Region.h"

/* Exported constants --------------------------------------------------------*/
   /*!
 * LoRaWAN confirmed messages
 */

#define LORAWAN_ADR_ON                              1
#define LORAWAN_ADR_OFF                             0
/* Exported types ------------------------------------------------------------*/


/*!
 * Application Data structure
 */
typedef struct
{
  /*point to the LoRa App data buffer*/
  uint8_t* Buff;
  /*LoRa App data buffer size*/
  uint8_t BuffSize;
  /*Port on which the LoRa App is data is sent/ received*/
  uint8_t Port;
  
} lora_AppData_t;

typedef enum 
{
  LORA_RESET = 0, 
  LORA_SET = !LORA_RESET
} LoraFlagStatus;

typedef enum 
{
  LORA_DISABLE = 0, 
  LORA_ENABLE = !LORA_DISABLE
} LoraState_t;

typedef enum 
{
  LORA_ERROR = -1, 
  LORA_SUCCESS = 0
} LoraErrorStatus;

typedef enum 
{
  LORAWAN_UNCONFIRMED_MSG = 0, 
  LORAWAN_CONFIRMED_MSG = !LORAWAN_UNCONFIRMED_MSG
} LoraConfirm_t;

typedef enum 
{
  LORA_TRUE = 0, 
  LORA_FALSE = !LORA_TRUE
} LoraBool_t;

/*!
 * LoRa State Machine states 
 */
typedef enum eTxEventType
{
/*!
 * @brief AppdataTransmition issue based on timer every TxDutyCycleTime
 */
    TX_ON_TIMER,
/*!
 * @brief AppdataTransmition external event plugged on OnSendEvent( )
 */
    TX_ON_EVENT
} TxEventType_t;


/*!
 * LoRa State Machine states 
 */
typedef struct sLoRaParam
{
/*!
 * @brief Activation state of adaptativeDatarate
 */
    bool AdrEnable;
/*!
 * @brief Uplink datarate, if AdrEnable is off
 */
    int8_t TxDatarate;
/*!
 * @brief Enable or disable a public network
 *
 */
    bool EnablePublicNetwork;
/*!
 * @brief Number of trials for the join request.
 */
    uint8_t NbTrials;

} LoRaParam_t;

/* Lora Main callbacks*/
typedef struct sLoRaMainCallback
{
/*!
 * @brief Get the current battery level
 *
 * @retval value  battery level ( 0: very low, 254: fully charged )
 */
    uint8_t ( *BoardGetBatteryLevel )( void );
/*!
 * \brief Get the current temperature
 *
 * \retval value  temperature in degreeCelcius( q7.8 )
 */
  uint16_t ( *BoardGetTemperatureLevel)( void );
/*!
 * @brief Gets the board 64 bits unique ID 
 *
 * @param [IN] id Pointer to an array that will contain the Unique ID
 */
    void    ( *BoardGetUniqueId ) ( uint8_t *id);
  /*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * @retval seed Generated pseudo random seed
 */
    uint32_t ( *BoardGetRandomSeed ) (void);
/*!
 * @brief Process Rx Data received from Lora network 
 *
 * @param [IN] AppData structure
 *
 */
    void ( *LORA_RxData ) ( lora_AppData_t *AppData);
    
    /*!
 * @brief callback indicating EndNode has jsu joiny 
 *
 * @param [IN] None
 */
    void ( *LORA_HasJoined)( void );
    /*!
 * @brief Confirms the class change 
 *
 * @param [IN] AppData is a buffer to process
 *
 * @param [IN] port is a Application port on wicth Appdata will be sent
 *
 * @param [IN] length is the number of recieved bytes
 */
    void ( *LORA_ConfirmClass) ( DeviceClass_t Class );
  
} LoRaMainCallback_t;



/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief Lora Initialisation
 * @param [IN] LoRaMainCallback_t
 * @param [IN] application parmaters
 * @retval none
 */
void LORA_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParam );

/**
 * @brief run Lora classA state Machine 
 * @param [IN] none
 * @retval none
 */
LoraErrorStatus LORA_send(lora_AppData_t* AppData, LoraConfirm_t IsTxConfirmed);

/**
 * @brief Join a Lora Network in classA
 * @Note if the device is ABP, this is a pass through functon
 * @param [IN] none
 * @retval none
 */
void LORA_Join( void);

/**
 * @brief Check whether the Device is joined to the network
 * @param [IN] none
 * @retval returns LORA_SET if joined
 */
LoraFlagStatus LORA_JoinStatus( void);

/**
 * @brief change Lora Class
 * @Note callback LORA_ConfirmClass informs upper layer that the change has occured
 * @Note Only switch from class A to class B/C OR from  class B/C to class A is allowed
 * @Attention can be calld only in LORA_ClassSwitchSlot or LORA_RxData callbacks
 * @param [IN] DeviceClass_t NewClass
 * @retval LoraErrorStatus
 */
LoraErrorStatus LORA_RequestClass( DeviceClass_t newClass );

/**
 * @brief get the current Lora Class
 * @param [IN] DeviceClass_t NewClass
 * @retval None
 */
void LORA_GetCurrentClass( DeviceClass_t *currentClass );
/**
  * @brief  Set join activation process: OTAA vs ABP
  * @param  Over The Air Activation status to set: enable or disable
  * @retval None
  */
void lora_config_otaa_set(LoraState_t otaa);

/**
  * @brief  Get join activation process: OTAA vs ABP
  * @param  None
  * @retval ENABLE if OTAA is used, DISABLE if ABP is used
  */
LoraState_t lora_config_otaa_get(void);

/**
  * @brief  Set duty cycle: ENABLE or DISABLE
  * @param  Duty cycle to set: enable or disable
  * @retval None
  */
void lora_config_duty_cycle_set(LoraState_t duty_cycle);

/**
  * @brief  Get Duty cycle: OTAA vs ABP
  * @param  None
  * @retval ENABLE / DISABLE
  */
LoraState_t lora_config_duty_cycle_get(void);

/**
 * @brief  Set whether or not acknowledgement is required
 * @param  ENABLE or DISABLE
 * @retval None
 */
void lora_config_reqack_set(LoraConfirm_t reqack);

/**
 * @brief  Get whether or not acknowledgement is required
 * @param  None
 * @retval ENABLE or DISABLE
 */
LoraConfirm_t lora_config_reqack_get(void);

/**
 * @brief  Get the SNR of the last received data
 * @param  None
 * @retval SNR
 */
int8_t lora_config_snr_get(void);

/**
 * @brief  Get the RSSI of the last received data
 * @param  None
 * @retval RSSI
 */
int16_t lora_config_rssi_get(void);

/**
 * @brief  Get whether or not the last sent data were acknowledged
 * @param  None
 * @retval ENABLE if so, DISABLE otherwise
 */
LoraState_t lora_config_isack_get(void);

/**
 * @brief  Launch LoraWan certification tests
 * @param  None
 * @retval The application port
 */ 
void lora_wan_certif(void);

/**
 * @brief  set tx datarate
 * @param  None
 * @retval The application port
 */ 
void lora_config_tx_datarate_set(int8_t TxDataRate);

/**
 * @brief  get tx datarate
 * @param  None
 * @retval tx datarate
 */ 
int8_t lora_config_tx_datarate_get(void );


/**
 * @brief  get linkcheck
 * @param  None
 * @retval linkcheck mode
 */
uint8_t lora_config_linkcheck_get(void);

/**
 * @brief  set linkcheck
 * @param  linkcheck mode
 * @retval None
 */
void lora_config_linkcheck_set(uint8_t LinkCheck);

/**
  * @brief  set 
  * @param  
  * @retval None
  */
void lora_config_application_port_set(uint8_t application_port);

/**
  * @brief  get 
  * @param  
  * @retval None
  */
uint8_t lora_config_application_port_get(void );

void Flash_store_key(void);
void Flash_read_key(void);

void Flash_Store_Config(void);
void Flash_Read_Config(void);

void key_printf(void);

uint32_t customize_freq1_get(void);

void customize_freq1_set(uint32_t Freq);

uint32_t customize_set8channel_get(void);

void customize_set8channel_set(uint8_t Freq);

void region_printf(void);
	
uint8_t Uplink_data_adaptive_rate(lora_AppData_t* AppData);

void lora_config_deveui_get(uint8_t *deveui);
void lora_config_deveui_set(uint8_t deveui[8]);
void lora_config_appeui_get(uint8_t *appeui);
void lora_config_appeui_set(uint8_t appeui[8]);
void lora_config_appkey_get(uint8_t *appkey);
void lora_config_appkey_set(uint8_t appkey[16]);
uint32_t lora_config_devaddr_get(void);
void lora_config_devaddr_set(uint32_t devaddr);
void lora_config_appskey_get(uint8_t *appskey);
void lora_config_appskey_set(uint8_t appskey[16]);
void lora_config_nwkskey_get(uint8_t *nwkskey);
void lora_config_nwkskey_set(uint8_t nwkskey[16]);
void LoRa_Join(void);
uint8_t AT_data_send(uint8_t confirm, uint8_t port, uint8_t *payload, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /*__LORA_APP_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
