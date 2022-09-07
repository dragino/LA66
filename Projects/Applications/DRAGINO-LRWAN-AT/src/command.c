/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include <stdlib.h>
#include <string.h>
#include "log.h"
#include "LoRaMac.h"
#include "Region.h"
#include "tremo_flash.h"
#include "tremo_delay.h"
#include "tremo_uart.h"
#include "tremo_system.h"

#include "command.h"
#include "lora_app.h"
#include "timer.h"
#include "LoRaMac.h"
#include "version.h"
#include "bsp.h"
#include "flash_eraseprogram.h"
#include "tremo_system.h"
#include "tremo_gpio.h"
#include "lora_config.h"
#include "radio.h"
#include "tremo_rcc.h"
#include "sx126x-board.h"

#define ARGC_LIMIT 16
#define ATCMD_SIZE (242 * 2 + 18)
#define PORT_LEN 4

#define QUERY_CMD		0x01
#define EXECUTE_CMD		0x02
#define DESC_CMD        0x03
#define SET_CMD			0x04

static char ReceivedData[255];
static unsigned ReceivedDataSize = 0;
static uint8_t ReceivedDataPort=0;

uint8_t dwelltime;
uint8_t parse_flag=0;
bool atrecve_flag=0;
bool debug_flags=0;
uint8_t atcmd[ATCMD_SIZE];
uint16_t atcmd_index = 0;
volatile bool g_atcmd_processing = false;
//uint8_t g_default_key[LORA_KEY_LENGTH] = {0x41, 0x53, 0x52, 0x36, 0x35, 0x30, 0x58, 0x2D, 
//                                          0x32, 0x30, 0x31, 0x38, 0x31, 0x30, 0x33, 0x30};

extern LoRaMainCallback_t *LoRaMainCallbacks;
extern char *band_string;
extern uint32_t LoRaMacState;
extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;

extern TimerEvent_t ReJoinTimer;
extern TimerEvent_t CalibrationUTCTimer;

extern TimerEvent_t DownlinkDetectTimeoutTimer;
extern TimerEvent_t UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer;

uint8_t symbtime1_value;
uint8_t symbtime2_value;
uint8_t flag1=0;
uint8_t flag2=0;
uint8_t exitintmode;
uint8_t exitmode;

extern __IO uint32_t write_address;
extern bool sleep_status;
uint16_t power_time=0;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;

extern bool FDR_status;

extern uint8_t currentLeapSecond;
extern uint8_t time_synchronization_method;
extern uint8_t time_synchronization_interval;
extern uint8_t pid_flag;
extern uint8_t RX2DR_setting_status;

extern uint8_t downlink_detect_switch;
extern uint16_t downlink_detect_timeout;

extern uint8_t confirmed_uplink_counter_retransmission_increment_switch;
extern uint8_t confirmed_uplink_retransmission_nbtrials;

extern uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
extern uint8_t LinkADR_NbTrans_retransmission_nbtrials;
extern uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout;

uint8_t write_key_in_flash_status=0,write_config_in_flash_status=0;

typedef struct {
	char *cmd;
	int (*fn)(int opt, int argc, char *argv[]);	
}at_cmd_t;

//AT functions
static int at_debug_func(int opt, int argc, char *argv[]);
static int at_reset_func(int opt, int argc, char *argv[]);
static int at_fdr_func(int opt, int argc, char *argv[]);
static int at_deui_func(int opt, int argc, char *argv[]);
static int at_appeui_func(int opt, int argc, char *argv[]);
static int at_appkey_func(int opt, int argc, char *argv[]);
static int at_devaddr_func(int opt, int argc, char *argv[]);
static int at_appskey_func(int opt, int argc, char *argv[]);
static int at_nwkskey_func(int opt, int argc, char *argv[]);
static int at_adr_func(int opt, int argc, char *argv[]);
static int at_txp_func(int opt, int argc, char *argv[]);
static int at_dr_func(int opt, int argc, char *argv[]);
static int at_dcs_func(int opt, int argc, char *argv[]);
static int at_pnm_func(int opt, int argc, char *argv[]);
static int at_rx2fq_func(int opt, int argc, char *argv[]);
static int at_rx2dr_func(int opt, int argc, char *argv[]);
static int at_rx1dl_func(int opt, int argc, char *argv[]);
static int at_rx2dl_func(int opt, int argc, char *argv[]);
static int at_jn1dl_func(int opt, int argc, char *argv[]);
static int at_jn2dl_func(int opt, int argc, char *argv[]);
static int at_njm_func(int opt, int argc, char *argv[]);
static int at_nwkid_func(int opt, int argc, char *argv[]);
static int at_fcu_func(int opt, int argc, char *argv[]);
static int at_fcd_func(int opt, int argc, char *argv[]);
static int at_class_func(int opt, int argc, char *argv[]);
static int at_join_func(int opt, int argc, char *argv[]);
static int at_njs_func(int opt, int argc, char *argv[]);
static int at_send_func(int opt, int argc, char *argv[]);
static int at_sendb_func(int opt, int argc, char *argv[]);
static int at_recv_func(int opt, int argc, char *argv[]);
static int at_recvb_func(int opt, int argc, char *argv[]);
static int at_ver_func(int opt, int argc, char *argv[]);
static int at_cfm_func(int opt, int argc, char *argv[]);
//static int at_cfs_func(int opt, int argc, char *argv[]);
static int at_snr_func(int opt, int argc, char *argv[]);
static int at_rssi_func(int opt, int argc, char *argv[]);
static int at_port_func(int opt, int argc, char *argv[]);

static int at_chs_func(int opt, int argc, char *argv[]);
#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
static int at_che_func(int opt, int argc, char *argv[]);
#endif

static int at_sleep_func(int opt, int argc, char *argv[]);
static int at_bat_func(int opt, int argc, char *argv[]);
static int at_cfg_func(int opt, int argc, char *argv[]);
#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )
static int at_dwellt_func(int opt, int argc, char *argv[]);
#endif
static int at_rjtdc_func(int opt, int argc, char *argv[]);
static int at_rpl_func(int opt, int argc, char *argv[]);
static int at_timestamp_func(int opt, int argc, char *argv[]);
static int at_leapsec_func(int opt, int argc, char *argv[]);
static int at_syncmod_func(int opt, int argc, char *argv[]);
static int at_synctdc_func(int opt, int argc, char *argv[]);
static int at_downlink_detect_func(int opt, int argc, char *argv[]);
static int at_setmaxnbtrans_func(int opt, int argc, char *argv[]);
static int at_devicetimereq_func(int opt, int argc, char *argv[]);
static int at_chsignaldetect_func(int opt, int argc, char *argv[]);

#ifdef LA66_HARDWARE_TEST
static int at_gpiotest_func(int opt, int argc, char *argv[]);
static int at_ctxcw_func(int opt, int argc, char *argv[]);
#endif

static at_cmd_t g_at_table[] = {
	  {AT_DEBUG, at_debug_func},
		{AT_RESET, at_reset_func},
		{AT_FDR, at_fdr_func},
	  {AT_DEUI, at_deui_func},
		{AT_APPEUI, at_appeui_func},
		{AT_APPKEY, at_appkey_func},
		{AT_DEVADDR, at_devaddr_func},
		{AT_APPSKEY, at_appskey_func},
		{AT_NWKSKEY, at_nwkskey_func},	
		{AT_ADR, at_adr_func},
		{AT_TXP, at_txp_func},
		{AT_DR, at_dr_func},
		{AT_DCS, at_dcs_func},
		{AT_PNM, at_pnm_func},
		{AT_RX2FQ, at_rx2fq_func},
		{AT_RX2DR, at_rx2dr_func},
		{AT_RX1DL, at_rx1dl_func},
		{AT_RX2DL, at_rx2dl_func},
		{AT_JN1DL, at_jn1dl_func},
		{AT_JN2DL, at_jn2dl_func},
		{AT_NJM, at_njm_func},
		{AT_NWKID, at_nwkid_func},
		{AT_FCU, at_fcu_func},
		{AT_FCD, at_fcd_func},
		{AT_CLASS, at_class_func},
		{AT_JOIN, at_join_func},
		{AT_NJS, at_njs_func},
		{AT_SENDB, at_sendb_func},
		{AT_SEND, at_send_func},
		{AT_RECVB,at_recvb_func},
		{AT_RECV,at_recv_func},
		{AT_VER, at_ver_func},
		{AT_CFM, at_cfm_func},
		{AT_SNR, at_snr_func},
		{AT_RSSI, at_rssi_func},
		{AT_PORT, at_port_func},
		{AT_CHS, at_chs_func},
		#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
		{AT_CHE, at_che_func},
		#endif

		{AT_SLEEP, at_sleep_func},
		{AT_BAT, at_bat_func},
		{AT_CFG, at_cfg_func},

		#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )
		{AT_DWELLT, at_dwellt_func},
		#endif
		{AT_RJTDC, at_rjtdc_func},
		{AT_RPL, at_rpl_func},
		{AT_TIMESTAMP, at_timestamp_func},
		{AT_LEAPSEC, at_leapsec_func},
		{AT_SYNCMOD, at_syncmod_func},
		{AT_SYNCTDC, at_synctdc_func},
		{AT_DDETECT, at_downlink_detect_func},
		{AT_SETMAXNBTRANS, at_setmaxnbtrans_func},
		{AT_DEVICETIMEREQ,at_devicetimereq_func},
		{AT_CHSIGNALDETECT,at_chsignaldetect_func},
		#ifdef LA66_HARDWARE_TEST
		{AT_GPIOTEST,at_gpiotest_func},
		{AT_CTXCW,at_ctxcw_func},
		#endif
};

#define AT_TABLE_SIZE	(sizeof(g_at_table) / sizeof(at_cmd_t))

uint8_t rxdata_change_hex(const char *hex, uint8_t *bin)
{
	uint8_t leg_temp=0;
  uint8_t flags=0;
	uint8_t *cur = bin;
  uint16_t hex_length = strlen(hex);
  const char *hex_end = hex + hex_length;
  uint8_t num_chars = 0;
  uint8_t byte = 0;
	
  while (hex < hex_end) {
        flags=0;
        if ('A' <= *hex && *hex <= 'F') {
            byte |= 10 + (*hex - 'A');
        } else if ('a' <= *hex && *hex <= 'f') {
            byte |= 10 + (*hex - 'a');
        } else if ('0' <= *hex && *hex <= '9') {
            byte |= *hex - '0';
        } else if (*hex == ' ') {
           flags=1;
        }else {
            return -1;
        }
        hex++;
        
        if(flags==0)
        {
          num_chars++;
          if (num_chars >= 2) {
              num_chars = 0;
              *cur++ = byte;
							leg_temp++;
              byte = 0;
             } else {
              byte <<= 4;
             }
        }
    }	
	
	 return leg_temp;
}

void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize)
{
  if (255 <= BuffSize)
    BuffSize = 255;
  memcpy1((uint8_t *)ReceivedData, Buff, BuffSize);
  ReceivedDataSize = BuffSize;
  ReceivedDataPort = AppPort;
}

static int hex2bin(const char *hex, uint8_t *bin, uint16_t bin_length)
{
	  uint8_t flags=0;
    uint16_t hex_length = strlen(hex);
    const char *hex_end = hex + hex_length;
    uint8_t *cur = bin;
    uint8_t num_chars = 0;
    uint8_t byte = 0;

    if ((hex_length + 1) / 2 > bin_length) {
       if((hex_length!=11)&&(hex_length!=23)&&(hex_length!=47))
       {
        return -1;
       }
    }

    while (hex < hex_end) {
        flags=0;
        if ('A' <= *hex && *hex <= 'F') {
            byte |= 10 + (*hex - 'A');
        } else if ('a' <= *hex && *hex <= 'f') {
            byte |= 10 + (*hex - 'a');
        } else if ('0' <= *hex && *hex <= '9') {
            byte |= *hex - '0';
        } else if (*hex == ' ') {
           flags=1;
        }else {
            return -1;
        }
        hex++;
        
        if(flags==0)
        {
          num_chars++;
          if (num_chars >= 2) {
              num_chars = 0;
              *cur++ = byte;
              byte = 0;
             } else {
              byte <<= 4;
             }
        }
    }
		
    return cur - bin;
}

static int hex2bin_format2(const char *hex, uint8_t *bin, uint16_t bin_length)
{
    uint16_t hex_length = strlen(hex);
    const char *hex_end = hex + hex_length;
    uint8_t *cur = bin;
    uint8_t num_chars = hex_length & 1;
    uint8_t byte = 0;

    if ((hex_length + 1) / 2 > bin_length) {
        return -1;
    }

    while (hex < hex_end) {
        if ('A' <= *hex && *hex <= 'F') {
            byte |= 10 + (*hex - 'A');
        } else if ('a' <= *hex && *hex <= 'f') {
            byte |= 10 + (*hex - 'a');
        } else if ('0' <= *hex && *hex <= '9') {
            byte |= *hex - '0';
        } else {
            return -1;
        }
        hex++;
        num_chars++;

        if (num_chars >= 2) {
            num_chars = 0;
            *cur++ = byte;
            byte = 0;
        } else {
            byte <<= 4;
        }
    }
    return cur - bin;
}

static int at_debug_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS;   
          debug_flags=1;
					snprintf((char *)atcmd, ATCMD_SIZE, "Enter Debug mode\r\n");
					
          break;
        }
		case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Set more info output\r\n");
			    break;
				}		
		
     default: break;
    }
    
    return ret;
}

static int at_reset_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS;   
          system_reset();
          break;
        }
				
		case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Trig a reset of the MCU\r\n");
			    break;
				}
		
     default: break;
    }
    
    return ret;
}

static int at_fdr_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS;   
					uint8_t status[128]={0};
					memset(status, 0x00, 128);
					
					flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
					if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
					{
						snprintf((char *)atcmd, ATCMD_SIZE, "FDR error\r\n");
					}
		
					delay_ms(100);
          system_reset();
          break;
        }
				
		case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Reset Parameters to Factory Default, Keys Reserve\r\n");
					break;
				}
		
     default: break;
    }
    
    return ret;
}

static int at_deui_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[8];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            lora_config_deveui_get(buf);                                                                                                          
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 8);
            if (length == 8) {
                    lora_config_deveui_set(buf);						
                    ret = LWAN_SUCCESS; 
                    write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Device EUI\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_appeui_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[8];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            lora_config_appeui_get(buf);	                                                                                                           
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 8);
            if (length == 8) {
                    lora_config_appeui_set(buf);
                    ret = LWAN_SUCCESS;
							      write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Application EUI\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_appkey_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[16];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            lora_config_appkey_get(buf);	                                                                                                            
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);								
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 16);
            if (length == 16) {
                    lora_config_appkey_set(buf);                  
                    ret = LWAN_SUCCESS;
                    write_key_in_flash_status=1;							
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Application Key\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_devaddr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint32_t devaddr;
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            devaddr=lora_config_devaddr_get();	                                                                                                             
            snprintf((char *)atcmd, ATCMD_SIZE, "%08X\r\n", (unsigned int)devaddr);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
					
            uint8_t buf[4];
            length = hex2bin((const char *)argv[0], buf, 4);
            if (length == 4) {
							      devaddr = buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3];
                    lora_config_devaddr_set(devaddr);
                    ret = LWAN_SUCCESS;
							      write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Device Address\r\n");
					break;
				}
        default: break;
    }
    
    return ret;
}

static int at_appskey_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[16];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            lora_config_appskey_get(buf);                                                                                                  
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 16);
            if (length == 16) {
                    lora_config_appskey_set(buf);
                    ret = LWAN_SUCCESS;   
							      write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Application Session Key\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_nwkskey_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[16];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            lora_config_nwkskey_get(buf);	                                                                                                           
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);            
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 16);
            if (length == 16) {
                    lora_config_nwkskey_set(buf);
                    ret = LWAN_SUCCESS; 
                    write_key_in_flash_status=1;							
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Network Session Key\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_adr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t adr;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_ADR;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.AdrEnable);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            adr = strtol((const char *)argv[0], NULL, 0);
            if(adr<2)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_ADR;
							  mib.Param.AdrEnable = adr;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
									
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Adaptive Data Rate setting. (0: off, 1: on)\r\n");
					break;
				}
				
        default: break;
    }

    return ret;
}

static int at_txp_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t power;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						
            
						mib.Type = MIB_CHANNELS_TX_POWER;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.ChannelsTxPower);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            power = strtol((const char *)argv[0], NULL, 0);
            if((power<16)||((power>=40)&&(power<=52)))
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_CHANNELS_TX_POWER;
							  mib.Param.ChannelsTxPower = power;
								status=LoRaMacMibSetRequestConfirm(&mib);
								
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;					
        }
						
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Transmit Power (0-5, MAX:0, MIN:5, according to LoRaWAN Spec)\r\n");
					break;
				}	
        default: break;
    }

    return ret;
}

static int at_dr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t datarate;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_CHANNELS_DATARATE;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.ChannelsDatarate);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            datarate = strtol((const char *)argv[0], NULL, 0);

						#if defined( REGION_AS923 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_AU915 )
						if((datarate==7)||(datarate>=14))
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_CN470 )
						if(datarate>=6)	
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_CN779 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_EU433 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_IN865 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_EU868 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;	
						}
				    #elif defined( REGION_KR920 )
						if(datarate>=6)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_US915 )
						if(((datarate>=5)&&(datarate<=7))||(datarate>=14))
						{
						ret = LWAN_PARAM_ERROR;break;								
						}
				    #elif defined( REGION_RU864 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}	
				    #elif defined( REGION_KZ865 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;							
						}			
				    #endif
		
						lora_config_tx_datarate_set(datarate) ;
						snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after AT+ADR=0\r\n");
						ret = LWAN_SUCCESS;              
						write_config_in_flash_status=1;
						
            break;
        }
						
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Data Rate. (0-7 corresponding to DR_X)\r\n");
					break;
				}	
        default: break;
    }

    return ret;
}

static int at_dcs_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
            if (lora_config_duty_cycle_get() == LORA_ENABLE)											
					  {
							status=1;
						}
						else
						{
							status=0;							
						}	
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", status);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
					  ret = LWAN_SUCCESS;
					  write_config_in_flash_status=1;
					  atcmd[0] = '\0';
					
            if(status==0)
            {
                lora_config_duty_cycle_set(LORA_DISABLE);								                
            }
						else if(status==1)
						{
							  lora_config_duty_cycle_set(LORA_ENABLE);
						}
            else
            {   
                write_config_in_flash_status=0;							
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the ETSI Duty Cycle setting - 0=disable, 1=enable - Only for testing\r\n");
					break;
				}	
        default: break;
    }

    return ret;
}
	
static int at_pnm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_PUBLIC_NETWORK;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.EnablePublicNetwork);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            if(status<2)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status1;
								mib.Type = MIB_PUBLIC_NETWORK;
							  mib.Param.EnablePublicNetwork = status;
								status1=LoRaMacMibSetRequestConfirm(&mib);
								
							  if(status1==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the public network mode. (0: off, 1: on)\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx2fq_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t freq;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RX2_CHANNEL;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.Rx2Channel.Frequency);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            freq = strtol((const char *)argv[0], NULL, 0);
            if(100000000<freq && freq<999000000)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_RX2_CHANNEL;
					      LoRaMacMibGetRequestConfirm(&mib);
							
								mib.Type = MIB_RX2_CHANNEL;
							
								mib.Param.Rx2Channel = ( Rx2ChannelParams_t ){ freq , mib.Param.Rx2Channel.Datarate };
					
								status=LoRaMacMibSetRequestConfirm(&mib);
								
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Rx2 window frequency\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx2dr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t dr;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RX2_CHANNEL;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.Rx2Channel.Datarate);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            dr = strtol((const char *)argv[0], NULL, 0);
            if(dr<16)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
						 	  mib.Type = MIB_RX2_CHANNEL;
					      LoRaMacMibGetRequestConfirm(&mib);
							
								mib.Type = MIB_RX2_CHANNEL;
							  mib.Param.Rx2Channel = ( Rx2ChannelParams_t ){ mib.Param.Rx2Channel.Frequency , dr };
								status=LoRaMacMibSetRequestConfirm(&mib);
								RX2DR_setting_status=1;
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Rx2 window data rate (0-7 corresponding to DR_X)\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx1dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RECEIVE_DELAY_1;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.ReceiveDelay1);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_RECEIVE_DELAY_1;
							  mib.Param.ReceiveDelay1 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
                atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the delay between the end of the Tx and the Rx Window 1 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx2dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RECEIVE_DELAY_2;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.ReceiveDelay2);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_RECEIVE_DELAY_2;
							  mib.Param.ReceiveDelay2 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the delay between the end of the Tx and the Rx Window 2 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_jn1dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						
      
						mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.JoinAcceptDelay1);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
							  mib.Param.JoinAcceptDelay1 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Join Accept Delay between the end of the Tx and the Join Rx Window 1 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_jn2dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.JoinAcceptDelay2);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
							  mib.Param.JoinAcceptDelay2 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Join Accept Delay between the end of the Tx and the Join Rx Window 2 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_njm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
            if (lora_config_otaa_get() == LORA_ENABLE)											
					  {
							status=1;
						}
						else
						{
							status=0;							
						}	
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", status);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
					  ret = LWAN_SUCCESS;
					  write_config_in_flash_status=1;
					  atcmd[0] = '\0';
					
            if(status==0)
            {
                lora_config_otaa_set(LORA_DISABLE);								                
            }
						else if(status==1)
						{
							  lora_config_otaa_set(LORA_ENABLE);
						}
            else
            {            
                write_config_in_flash_status=0;							
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Network Join Mode. (0: ABP, 1: OTAA)\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_nwkid_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	  uint8_t length;
    uint8_t buf[4];
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_NET_ID;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X\r\n", (uint8_t)(mib.Param.NetID>>24&0xff),(uint8_t)(mib.Param.NetID>>16&0xff),(uint8_t)(mib.Param.NetID>>8&0xff),(uint8_t)(mib.Param.NetID&0xff));

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 4);
            if(length==4)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
								mib.Type = MIB_NET_ID;
							  mib.Param.NetID = buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3];
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Network ID\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_fcu_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t counter;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_UPLINK_COUNTER;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.UpLinkCounter);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            counter = strtol((const char *)argv[0], NULL, 0);
            if(counter>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_UPLINK_COUNTER;
							  mib.Param.UpLinkCounter = counter;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Frame Counter Uplink\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_fcd_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t counter;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_DOWNLINK_COUNTER;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.DownLinkCounter);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            counter = strtol((const char *)argv[0], NULL, 0);
            if(counter>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_DOWNLINK_COUNTER;
							  mib.Param.DownLinkCounter = counter;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Frame Counter Downlink\r\n");
					break;
				}
        default: break;
    }

    return ret;
}
	
static int at_class_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_DEVICE_CLASS;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%c\r\n", 'A' + mib.Param.Class);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;            
					
					  MibRequestConfirm_t mib;						
            LoRaMacStatus_t status;
					
						mib.Type = MIB_DEVICE_CLASS;
					
            switch (*argv[0])
						{
							case 'A':
							case 'B':
							case 'C':
								/* assume CLASS_A == 0, CLASS_B == 1, etc, which is the case for now */
								mib.Param.Class = (DeviceClass_t)(*argv[0] - 'A');
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
								break;
							default:
								return LWAN_PARAM_ERROR;
						} 

            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Device Class\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_join_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS; 
          atcmd[0] = '\0';					
          LORA_Join();
          break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Join network\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_njs_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_NETWORK_JOINED;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.IsNetworkJoined);
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the join status\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_send_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
					
            uint8_t confirm_status=0, fport=0,len=0;
					  uint16_t length=0;
					
            confirm_status = strtol((const char *)argv[0], NULL, 0);
					  fport = strtol((const char *)argv[1], NULL, 0);
					  length = strtol((const char *)argv[2], NULL, 0);
            len=strlen((const char *)argv[3]);
					
            if((len>0 && len<243) && (len==length))
            {
							  #if defined( ACCUHEALTH_LA66 )
							  gpio_init(LED_RGB_PORT, LED_RECV_DOWNLINK_PIN, GPIO_MODE_OUTPUT_PP_LOW);
							  #endif
							
								if ( LORA_JoinStatus () != LORA_SET)
								{
									/*Not joined, try again later*/
									return LWAN_NO_NET_JOINED;
								}
								
                uint8_t status=AT_data_send(confirm_status,fport,(uint8_t *)argv[3],length);
							
							  if(status==1)
								{
									ret = LWAN_SUCCESS;
								}
								else if(status==2)
								{
									ret = LWAN_BUSY_ERROR;
								}
                else
                 		ret = LWAN_PARAM_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Send text data along with the application port and confirm status\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_sendb_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
					
            uint8_t confirm_status=0, fport=0,len=0;
					  uint16_t length=0;
					  uint8_t payload[242];
					
            confirm_status = strtol((const char *)argv[0], NULL, 0);
					  fport = strtol((const char *)argv[1], NULL, 0);
					  length = strtol((const char *)argv[2], NULL, 0);
					  len = hex2bin_format2((const char *)argv[3], payload, length);
					
            if((len>0 && len<243) && (len==length))
            {           
							  #if defined( ACCUHEALTH_LA66 )
                gpio_init(LED_RGB_PORT, LED_RECV_DOWNLINK_PIN, GPIO_MODE_OUTPUT_PP_LOW);
							  #endif
							
								if ( LORA_JoinStatus () != LORA_SET)
								{
									/*Not joined, try again later*/
									return LWAN_NO_NET_JOINED;
								}
								
							  uint8_t status=AT_data_send(confirm_status,fport,payload,len);
							
								if(status==1)
								{
									ret = LWAN_SUCCESS;
								}
								else if(status==2)
								{
									ret = LWAN_BUSY_ERROR;
								}
                else
                 		ret = LWAN_PARAM_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Send hexadecimal data along with the application port and confirm status\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_recv_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS; 
						if(atrecve_flag==0)
						{
							#if defined( ACCUHEALTH_LA66 )
							gpio_init(LED_RGB_PORT, LED_RECV_DOWNLINK_PIN, GPIO_MODE_OUTPUT_PP_LOW);
							#endif
							
							LOG_PRINTF(LL_DEBUG,"%d:",ReceivedDataPort);
 				
						  if (ReceivedDataSize)
							{
								LOG_PRINTF(LL_DEBUG,"%s", ReceivedData);
								ReceivedDataSize = 0;
							}							
						}		
					  snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Print last received data in raw format\r\n");
					break;
				}
								
        default: break;
    }

    return ret;		
}

static int at_recvb_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS; 
						if(atrecve_flag==0)
						{
							#if defined( ACCUHEALTH_LA66 )
							gpio_init(LED_RGB_PORT, LED_RECV_DOWNLINK_PIN, GPIO_MODE_OUTPUT_PP_LOW);
							#endif
							
							LOG_PRINTF(LL_DEBUG,"%d:",ReceivedDataPort);
 				
							for (uint8_t i = 0; i < ReceivedDataSize; i++)
							{
								LOG_PRINTF(LL_DEBUG,"%02x", ReceivedData[i]);
							}
							ReceivedDataSize = 0;
						}		
					  snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");										
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Print last received data in binary format (with hexadecimal values)\r\n");
					break;
				}
								
        default: break;
    }

    return ret;		
}

static int at_ver_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;					 											

            snprintf((char *)atcmd, ATCMD_SIZE, "%s %s\r\n",band_string,AT_VERSION_STRING);
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get current image version and Frequency Band\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_cfm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode=0,trials=7,increment_switch=0;
	 
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d\r\n", lora_config_reqack_get(),confirmed_uplink_retransmission_nbtrials,confirmed_uplink_counter_retransmission_increment_switch);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
					
            ret = LWAN_SUCCESS;
					  atcmd[0] = '\0';
					
					  if(argc==1)
						{
							mode = strtol((const char *)argv[0], NULL, 0);
							trials=7;
			        increment_switch=0;
							write_config_in_flash_status=1;
						}
						else if(argc==3)
						{
							mode = strtol((const char *)argv[0], NULL, 0);
							trials=strtol((const char *)argv[1], NULL, 0);
			        increment_switch=strtol((const char *)argv[2], NULL, 0);
							write_config_in_flash_status=1;
						}											  
						
						if(mode>1 || trials>7 || increment_switch>1)
						{
							write_config_in_flash_status=0;
							return LWAN_PARAM_ERROR;
						}
						
						if(mode==0)
						{
							lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
							confirmed_uplink_retransmission_nbtrials=trials;
							confirmed_uplink_counter_retransmission_increment_switch=0;
						}
						else if(mode==1)
						{
							lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
							confirmed_uplink_retransmission_nbtrials=trials;
							confirmed_uplink_counter_retransmission_increment_switch=increment_switch;
						}
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the confirmation mode (0-1)\r\n");
					break;
				}
        default: break;
    }
    
    return ret;
}

static int at_snr_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;					 											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", lora_config_snr_get());
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the SNR of the last received packet\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_rssi_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;					 											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", lora_config_rssi_get());
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the RSSI of the last received packet\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_port_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t port=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", lora_config_application_port_get());

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            port = strtol((const char *)argv[0], NULL, 0);
            if(port<=255)
            {
                lora_config_application_port_set(port);
							  
                ret = LWAN_SUCCESS;
							  write_config_in_flash_status=1;
							  atcmd[0] = '\0';
            }
            else
            {              
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the application port\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_chs_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t freq=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)customize_freq1_get());

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            freq = strtol((const char *)argv[0], NULL, 0);
            
					  if((100000000<freq && freq<999999999) || freq==0)
						{
							customize_freq1_set(freq);
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Frequency (Unit: Hz) for Single Channel Mode\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
static int at_che_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;

		uint8_t fre;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;					 
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)customize_set8channel_get());

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            fre = strtol((const char *)argv[0], NULL, 0);
            
						#if defined ( REGION_CN470 )
						if((fre>12)||((fre>=1)&&(fre<=10)))
						{
							fre=11;
							snprintf((char *)atcmd, ATCMD_SIZE, "Error Subband, must be 0 or 11,12\r\n");	
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");	
						}
						#elif defined ( REGION_US915 )
						if(fre>8)
						{
							fre=1;
							snprintf((char *)atcmd, ATCMD_SIZE, "Error Subband, must be 0 ~ 8\r\n");
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");	
						}	
						#elif defined ( REGION_AU915 )
						if(fre>8)
						{
							fre=1;
							snprintf((char *)atcmd, ATCMD_SIZE, "Error Subband, must be 0 ~ 8\r\n");		
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");	
						}
						#else
						fre=0;
						#endif
						
						customize_set8channel_set(fre);
							
						ret= LWAN_SUCCESS;
            write_config_in_flash_status=1;
						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set eight channels mode,Only for US915,AU915,CN470\r\n");
					break;
				}
        default: break;
    }

    return ret;
}	
#endif

static int at_sleep_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {		
		 case QUERY_CMD: {
				ret = LWAN_SUCCESS;					 
			 
				snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", sleep_status);

			 break;
		}
				 
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS; 
					TimerStop(&MacStateCheckTimer);
					TimerStop(&TxDelayedTimer);
					TimerStop(&AckTimeoutTimer);

					TimerStop(&RxWindowTimer1);
					TimerStop(&RxWindowTimer2);

					TimerStop(&ReJoinTimer);
					TimerStop( &CalibrationUTCTimer);

					TimerStop( &DownlinkDetectTimeoutTimer);
					TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
					
					sleep_status=1;
					
					snprintf((char *)atcmd, ATCMD_SIZE, "SLEEP\r\n");
					
          break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Set sleep mode\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_bat_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint16_t bat;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					  bat=battery_voltage_measurement();
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", bat);

					 break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the current battery voltage in mV\r\n");
					break;
				}               
        default: break;
    }

    return ret;
}

static int at_cfg_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:		
		      {
          ret = LWAN_SUCCESS; 
					atcmd[0] = '\0';
					
					if(sleep_status==0)
					{
						if(LORA_JoinStatus () == LORA_SET)
						{
							if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
							{
								return LWAN_BUSY_ERROR;
							}
							else
							{
								LOG_PRINTF(LL_DEBUG,"Stop Tx events,Please wait for the erase to complete\r");
							}
						}
						else
						{
							LOG_PRINTF(LL_DEBUG,"Stop Tx events,Please wait for the erase to complete\r");
						}
					}					
					
					TimerStop(&MacStateCheckTimer);
					TimerStop(&TxDelayedTimer);
					TimerStop(&AckTimeoutTimer);

					TimerStop(&RxWindowTimer1);
					TimerStop(&RxWindowTimer2);
					//TimerStop(&TxTimer);
					TimerStop(&ReJoinTimer);
					TimerStop( &CalibrationUTCTimer);
					//TimerStop( &sht20tempcompTimer);
					
					atrecve_flag=1;						
					for (uint8_t num = 0; num < AT_TABLE_SIZE; num++)
					{							
						if(g_at_table[num].fn(QUERY_CMD, 0, 0)==LWAN_SUCCESS)
						{
							LOG_PRINTF(LL_DEBUG,"AT%s=",g_at_table[num].cmd);
							if(strcmp(g_at_table[num].cmd,AT_RECVB)==0)
							{
								atrecve_flag=0;
								g_at_table[num].fn(QUERY_CMD, 0, 0);
								atrecve_flag=1;
							}
							linkwan_serial_output(atcmd, strlen((const char *)atcmd));  
						}
						atcmd_index = 0;
						memset(atcmd, 0xff, ATCMD_SIZE);
					}    
					atrecve_flag=0;
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");
						
					delay_ms(1000);
					
					#if !defined( USB_LORAWAN_ADAPTER_LBT )
						if(sleep_status==0)
						{
								if(LORA_JoinStatus () == LORA_SET)
								{
									LOG_PRINTF(LL_DEBUG,"Start Tx events\r");
								}
								else
								{
									if(FDR_status==0)
									{
										LOG_PRINTF(LL_DEBUG,"Start Tx events\r");
										TimerStart(&TxDelayedTimer);
									}
								}
							
						}
					#endif
						
          break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Print all configurations\r\n");
					break;
				}
     default: break;
    }
    
    return ret;    
}

#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )
static int at_dwellt_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", dwelltime);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            
					  if(status<2)
						{
							dwelltime=status;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set UplinkDwellTime\r\n");
					break;
				}
        default: break;
    }

    return ret;
}
#endif

static int at_rjtdc_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint16_t interval;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", REJOIN_TX_DUTYCYCLE);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            interval = strtol((const char *)argv[0], NULL, 0);
            
					  if(interval>0 && interval<=65535)
						{
							REJOIN_TX_DUTYCYCLE=interval;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the ReJoin data transmission interval in min\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rpl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t level;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", response_level);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            level = strtol((const char *)argv[0], NULL, 0);
            
					  if(level<=4)
						{
							response_level=level;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set response level\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_timestamp_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
						struct tm localtime;
						SysTime_t sysTimeCurrent = { 0 };
						
						sysTimeCurrent=SysTimeGet();	
						SysTimeLocalTime( sysTimeCurrent.Seconds, &localtime );                     
            
            snprintf((char *)atcmd, ATCMD_SIZE, "systime= %d/%d/%d %02d:%02d:%02d (%u)\n\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,(unsigned int)sysTimeCurrent.Seconds);
            break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            uint32_t timestamp;
            SysTime_t sysTime = { 0 };
            
            timestamp=strtol((const char *)argv[0], NULL, 0);
            
            if (timestamp > 1640044800) {              
							sysTime.Seconds=timestamp;
		          SysTimeSet( sysTime );	
              ret = LWAN_SUCCESS;   
              atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set UNIX timestamp in second\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_leapsec_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t second;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", currentLeapSecond);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            second = strtol((const char *)argv[0], NULL, 0);
            
					  if(second<=255)
						{
							currentLeapSecond=second;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Leap Second\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_syncmod_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", time_synchronization_method);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode<2)
						{
							time_synchronization_method=mode;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set time synchronization method\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_synctdc_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t interval;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", time_synchronization_interval);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            interval = strtol((const char *)argv[0], NULL, 0);
            
					  if(interval>0 && interval<=255)
						{
							time_synchronization_interval=interval;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set time synchronization interval in day\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_downlink_detect_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	  uint8_t status;
	  uint16_t timeout1=0,timeout2=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d\r\n", downlink_detect_switch,unconfirmed_uplink_change_to_confirmed_uplink_timeout,downlink_detect_timeout);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            timeout1 = strtol((const char *)argv[1], NULL, 0);
					  timeout2 = strtol((const char *)argv[2], NULL, 0);
					
					  if(status<2)
						{
							downlink_detect_switch=status;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							return LWAN_PARAM_ERROR;

						if(timeout1>0 && timeout1<=65535 && timeout2>0 && timeout2<=65535)
						{
							unconfirmed_uplink_change_to_confirmed_uplink_timeout=timeout1;
							downlink_detect_timeout=timeout2;
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
						{	
              write_config_in_flash_status=0;							
							return LWAN_PARAM_ERROR;	
						}
	
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the downlink detection\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_setmaxnbtrans_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;

	  uint8_t trials,increment_switch;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", LinkADR_NbTrans_retransmission_nbtrials,LinkADR_NbTrans_uplink_counter_retransmission_increment_switch);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            trials = strtol((const char *)argv[0], NULL, 0);
            increment_switch = strtol((const char *)argv[1], NULL, 0);				  
					
						if(trials<1 || trials>15 || increment_switch>1)
						{
							return LWAN_PARAM_ERROR;
						}

						LinkADR_NbTrans_retransmission_nbtrials=trials;
						LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=increment_switch;

						ret = LWAN_SUCCESS;
						write_config_in_flash_status=1;
						atcmd[0] = '\0';

            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the max nbtrans in LinkADR\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_devicetimereq_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status=0;
    
	  uint8_t AppDataBuff[242];
	
    lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
		
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);          						
							
							if(status==1)
							{
								ret = LWAN_SUCCESS;
								atcmd[0] = '\0';
								
								if ( LORA_JoinStatus () != LORA_SET)
								{
									/*Not joined, try again later*/
									return LWAN_NO_NET_JOINED;
								}
							
								MlmeReq_t mlmeReq;
								mlmeReq.Type = MLME_DEVICE_TIME;
								LoRaMacMlmeRequest( &mlmeReq );								
								
								AppData.Port=4;								
						
								AppData.Buff[0]=0x00;
								
								AppData.BuffSize = 1;
								LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);								
							}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Device time req\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_chsignaldetect_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;

    uint32_t freq;
	  uint32_t timeout;
    int16_t rssi;
	
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
            
            freq = strtol((const char *)argv[0], NULL, 0);
					  rssi = strtol((const char *)argv[1], NULL, 0);
					  timeout = strtol((const char *)argv[2], NULL, 0);
					  atcmd[0] = '\0';
					
					  if(Radio.IsChannelFree( MODEM_FSK, freq, rssi, timeout )==true)
            {
							snprintf((char *)atcmd, ATCMD_SIZE, "FREE\r\n");
						}							
						else
							snprintf((char *)atcmd, ATCMD_SIZE, "BUSY\r\n");
						
						ret = LWAN_SUCCESS;						
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "FSK signal detection\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

#ifdef LA66_HARDWARE_TEST
static int at_gpiotest_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode==1)
						{			
							rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
							rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
							rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
							rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
							
							gpio_init(GPIOA, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP_LOW);							
							gpio_init(GPIOA, GPIO_PIN_6, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOA, GPIO_PIN_7, GPIO_MODE_INPUT_FLOATING);
							
							gpio_init(GPIOA, GPIO_PIN_14, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOA, GPIO_PIN_15, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOB, GPIO_PIN_7, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOB, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOB, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP_LOW);
							
							gpio_init(GPIOB, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOB, GPIO_PIN_10, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOB, GPIO_PIN_13, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOB, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOB, GPIO_PIN_14, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOB, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOC, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOD, GPIO_PIN_14, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOD, GPIO_PIN_10, GPIO_MODE_INPUT_FLOATING);
							
							gpio_init(GPIOA, GPIO_PIN_12, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOA, GPIO_PIN_13, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOA, GPIO_PIN_9, GPIO_MODE_INPUT_FLOATING);	
							
							gpio_init(GPIOA, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOA, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOA, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP_LOW);		
							gpio_init(GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP_LOW);
							gpio_init(GPIOC, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP_LOW);
							
							gpio_init(GPIOC, GPIO_PIN_8, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOC, GPIO_PIN_5, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOC, GPIO_PIN_1, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOC, GPIO_PIN_0, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOC, GPIO_PIN_4, GPIO_MODE_INPUT_FLOATING);
							gpio_init(GPIOC, GPIO_PIN_3, GPIO_MODE_INPUT_FLOATING);
							
							gpio_write(GPIOA, GPIO_PIN_1, 1);
							gpio_write(GPIOA, GPIO_PIN_0, 0);
							gpio_write(GPIOA, GPIO_PIN_3, 1);							
							gpio_write(GPIOB, GPIO_PIN_9, 0);
							gpio_write(GPIOB, GPIO_PIN_8, 1);
							gpio_write(GPIOB, GPIO_PIN_11, 0);
							gpio_write(GPIOB, GPIO_PIN_10, 1);		
							gpio_write(GPIOB, GPIO_PIN_12, 0);
							gpio_write(GPIOB, GPIO_PIN_14, 1);
							gpio_write(GPIOB, GPIO_PIN_15, 0);							
							gpio_write(GPIOC, GPIO_PIN_12, 1);
							gpio_write(GPIOC, GPIO_PIN_13, 0);			
              gpio_write(GPIOA, GPIO_PIN_11, 1);
              gpio_write(GPIOA, GPIO_PIN_8, 0);
							gpio_write(GPIOA, GPIO_PIN_5, 1);
							gpio_write(GPIOA, GPIO_PIN_4, 0);							
							gpio_write(GPIOC, GPIO_PIN_9, 1);							
							LOG_PRINTF(LL_DEBUG,"%d%d%d%d%d",gpio_read(GPIOA, GPIO_PIN_14),gpio_read(GPIOA, GPIO_PIN_15),gpio_read(GPIOB, GPIO_PIN_7),gpio_read(GPIOA, GPIO_PIN_6),gpio_read(GPIOA, GPIO_PIN_7));
							LOG_PRINTF(LL_DEBUG,"%d%d",gpio_read(GPIOB, GPIO_PIN_13),gpio_read(GPIOC, GPIO_PIN_3));
							LOG_PRINTF(LL_DEBUG,"%d%d%d%d%d",gpio_read(GPIOD, GPIO_PIN_14),gpio_read(GPIOD, GPIO_PIN_10),gpio_read(GPIOA, GPIO_PIN_12),gpio_read(GPIOA, GPIO_PIN_13),gpio_read(GPIOA, GPIO_PIN_9));							
							LOG_PRINTF(LL_DEBUG,"%d%d%d%d%d\n\r",gpio_read(GPIOC, GPIO_PIN_8),gpio_read(GPIOC, GPIO_PIN_5),gpio_read(GPIOC, GPIO_PIN_1),gpio_read(GPIOC, GPIO_PIN_0),gpio_read(GPIOC, GPIO_PIN_4));							
							
							gpio_write(GPIOA, GPIO_PIN_1, 0);
							gpio_write(GPIOA, GPIO_PIN_0, 1);
							gpio_write(GPIOA, GPIO_PIN_3, 0);							
							gpio_write(GPIOB, GPIO_PIN_9, 1);
							gpio_write(GPIOB, GPIO_PIN_8, 0);
							gpio_write(GPIOB, GPIO_PIN_11, 1);
							gpio_write(GPIOB, GPIO_PIN_10, 0);
							gpio_write(GPIOB, GPIO_PIN_12, 1);
							gpio_write(GPIOB, GPIO_PIN_14, 0);
							gpio_write(GPIOB, GPIO_PIN_15, 1);							
							gpio_write(GPIOC, GPIO_PIN_12, 0);
							gpio_write(GPIOC, GPIO_PIN_13, 1);
              gpio_write(GPIOA, GPIO_PIN_11, 0);
              gpio_write(GPIOA, GPIO_PIN_8, 1);
							gpio_write(GPIOA, GPIO_PIN_5, 0);
							gpio_write(GPIOA, GPIO_PIN_4, 1);							
							gpio_write(GPIOC, GPIO_PIN_9, 0);		
							
							LOG_PRINTF(LL_DEBUG,"%d%d%d%d%d",gpio_read(GPIOA, GPIO_PIN_14),gpio_read(GPIOA, GPIO_PIN_15),gpio_read(GPIOB, GPIO_PIN_7),gpio_read(GPIOA, GPIO_PIN_6),gpio_read(GPIOA, GPIO_PIN_7));
							LOG_PRINTF(LL_DEBUG,"%d%d",gpio_read(GPIOB, GPIO_PIN_13),gpio_read(GPIOC, GPIO_PIN_3));
							LOG_PRINTF(LL_DEBUG,"%d%d%d%d%d",gpio_read(GPIOD, GPIO_PIN_14),gpio_read(GPIOD, GPIO_PIN_10),gpio_read(GPIOA, GPIO_PIN_12),gpio_read(GPIOA, GPIO_PIN_13),gpio_read(GPIOA, GPIO_PIN_9));							
							LOG_PRINTF(LL_DEBUG,"%d%d%d%d%d\n\r",gpio_read(GPIOC, GPIO_PIN_8),gpio_read(GPIOC, GPIO_PIN_5),gpio_read(GPIOC, GPIO_PIN_1),gpio_read(GPIOC, GPIO_PIN_0),gpio_read(GPIOC, GPIO_PIN_4));																										
							
							ret = LWAN_SUCCESS;
							
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "GPIO test\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_ctxcw_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
            
						uint8_t opt   = 0;
						uint32_t freq = strtol(argv[0], NULL, 0);
						uint8_t pwr   = strtol(argv[1], NULL, 0);
						if (argc > 2)
								opt = strtol(argv[2], NULL, 0);
						
            SX126xSetPaOpt(opt);
						Radio.SetTxContinuousWave(freq, pwr, 0xffff);
								
					  ret = LWAN_SUCCESS;
							
						atcmd[0] = '\0';
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "tx test\r\n");
					break;
				}
        default: break;
    }

    return ret;
}
#endif

// this can be in intrpt context
void linkwan_serial_input(uint8_t cmd)
{
    if(g_atcmd_processing) 
        return;
    
    if ((cmd >= '0' && cmd <= '9') || (cmd >= 'a' && cmd <= 'z') ||
        (cmd >= 'A' && cmd <= 'Z') || cmd == '?' || cmd == '+' ||
        cmd == ':' || cmd == '=' || cmd == ' ' || cmd == ',' || cmd == '-') {
        if (atcmd_index >= ATCMD_SIZE) {
            memset(atcmd, 0xff, ATCMD_SIZE);
            atcmd_index = 0;
            return;
        }
        atcmd[atcmd_index++] = cmd;
    } else if (cmd == '\r' || cmd == '\n') {
        if (atcmd_index >= ATCMD_SIZE) {
            memset(atcmd, 0xff, ATCMD_SIZE);
            atcmd_index = 0;
            return;
        }
        atcmd[atcmd_index] = '\0';
    }
}

int linkwan_serial_output(uint8_t *buffer, int len)
{
    LOG_PRINTF(LL_DEBUG,"%s", buffer);

    return 0;
}

void linkwan_at_process(void)
{
    char *ptr = NULL;
		int argc = 0;
		int index = 0;
		char *argv[ARGC_LIMIT];
			int ret = LWAN_ERROR;
			uint8_t *rxcmd = atcmd + 2;
			int16_t rxcmd_index = atcmd_index - 2;

			if (atcmd_index <=2 && atcmd[atcmd_index] == '\0') {
					
					atcmd_index = 0;
					memset(atcmd, 0xff, ATCMD_SIZE);
					return;
			}
			
			if (rxcmd_index <= 0 || rxcmd[rxcmd_index] != '\0') {
					return;
			}

			g_atcmd_processing = true;
			
			if(atcmd[0] != 'A' || atcmd[1] != 'T')
					goto at_end;
			
			if((atcmd[0] == 'A')&&(atcmd[1] == 'T')&&(atcmd[2] == '?')&&(atcmd[3] == '\0'))  //AT?
			{
        for (uint8_t num = 0; num < AT_TABLE_SIZE; num++)
        {							
          if(g_at_table[num].fn(DESC_CMD, 0, 0)==LWAN_SUCCESS)
          {
				  	LOG_PRINTF(LL_DEBUG,"AT%s: ",g_at_table[num].cmd);
            linkwan_serial_output(atcmd, strlen((const char *)atcmd));  
          }
					delay_ms(50);
          atcmd_index = 0;
          memset(atcmd, 0xff, ATCMD_SIZE);
         }	
				 snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");
				 ret=LWAN_SUCCESS;			
			}
			else
			{
			for (index = 0; index < AT_TABLE_SIZE; index++) {
					int cmd_len = strlen(g_at_table[index].cmd);
				if (!strncmp((const char *)rxcmd, g_at_table[index].cmd, cmd_len)) {
					ptr = (char *)rxcmd + cmd_len;
					break;
				}
			}
		if (index >= AT_TABLE_SIZE || !g_at_table[index].fn)
					goto at_end;

    if (ptr[0] == '\0') {
			ret = g_at_table[index].fn(EXECUTE_CMD, argc, argv);
		}  else if (ptr[0] == ' ') {
					argv[argc++] = ptr;
			ret = g_at_table[index].fn(EXECUTE_CMD, argc, argv);
		} 
		else if( (ptr[0] == '?') && (ptr[1] == '\0')) {
				ret = g_at_table[index].fn(DESC_CMD, argc, argv);				
		}else if ((ptr[0] == '=') && (ptr[1] == '?') && (ptr[2] == '\0')) {
					ret = g_at_table[index].fn(QUERY_CMD, argc, argv);
		} else if (ptr[0] == '=') {
			ptr += 1;
					
					char *str = strtok((char *)ptr, ",");
					while(str) {
							argv[argc++] = str;
							str = strtok((char *)NULL, ",");
					}
			ret = g_at_table[index].fn(SET_CMD, argc, argv);
					
			if(ret==LWAN_SUCCESS && write_key_in_flash_status==1)
			{
				write_key_in_flash_status=0;
				Flash_store_key();
			}
			
			if(ret==LWAN_SUCCESS && write_config_in_flash_status==1)
			{
				write_config_in_flash_status=0;
				Flash_Store_Config();
			}
			
		} else {
			ret = LWAN_ERROR;
		}
	 }
			
	at_end:

			if (LWAN_ERROR == ret)
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_ERROR);
			else if(LWAN_PARAM_ERROR == ret)
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_PARAM_ERROR);
			else if(LWAN_BUSY_ERROR == ret) 
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_BUSY_ERROR);
			else if(LWAN_NO_NET_JOINED == ret) 
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_NO_NET_JOINED);
			

			linkwan_serial_output(atcmd, strlen((const char *)atcmd));  	
	 
			if(ret==LWAN_SUCCESS)
			{
				LOG_PRINTF(LL_DEBUG,"\r\nOK\r\n");
			}	  
	 
    atcmd_index = 0;
    memset(atcmd, 0xff, ATCMD_SIZE);
    g_atcmd_processing = false;        
    return;
}





void linkwan_at_init(void)
{
    atcmd_index = 0;
    memset(atcmd, 0xff, ATCMD_SIZE);
}