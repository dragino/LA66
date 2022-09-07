#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "log.h"
#include "tremo_uart.h"
#include "tremo_lpuart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_iwdg.h"
#include "tremo_delay.h"
#include "tremo_pwr.h"
#include "rtc-board.h"
#include "lora_app.h"
#include "bsp.h"
#include "command.h"
#include "lora_config.h"
#include "I2C_A.h"
#include "ds18b20.h"
#include "flash_eraseprogram.h"
#include "tremo_rtc.h"
#include "tremo_system.h"
#include "tremo_adc.h"
#include "sx126x.h"
#include "tremo_flash.h"

log_level_t g_log_level = LL_ERR | LL_WARN | LL_DEBUG;

extern bool print_isdone(void);

extern SysTime_t LastTxdoneTime;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*!
 * Defines the application data transmission duty cycle. 60s, value in [ms].
 */

/*
 *Poll Message Flag: 1: This message is a poll message from server, 0: means this is a normal uplink.
 *Data Record Flag: 1: this message is stored in Flash,0: This message is not stored in flash
 *UTC Set time OK: 1: Set time ok,0: N/A
 *UTC Set Time Request:1: Request server downlink UTC time, 0 : N/A
 */
bool Poll_Message_Flag=0,Data_Record_Flag=0,UTC_Accept=0,UTC_Request=1;

bool sleep_status=0;//AT+SLEEP
bool is_lora_joined=0;

bool is_time_to_IWDG_Refresh=0;

extern bool debug_flags;

uint8_t is_PDTA_command=0;
uint8_t is_PLDTA_command=0;

extern uint8_t dwelltime;
uint8_t atz_flags=0;
uint16_t REJOIN_TX_DUTYCYCLE=20;//min
bool MAC_COMMAND_ANS_status=0;
uint8_t response_level=0;
bool is_there_data=0;
bool rejoin_status=0;
bool rejoin_keep_status=0;

bool is_time_to_rejoin=0;
bool JoinReq_NbTrails_over=0;
bool unconfirmed_downlink_data_ans_status=0,confirmed_downlink_data_ans_status=0;

__IO uint8_t EnterLowPowerStopModeStatus=0,EnterLowPowerStopMode_error_times=0;

uint8_t is_lora_joined_keep_status=0;

extern uint8_t printf_dma_busy;
extern uint16_t printf_dma_idx_w;
extern uint16_t printf_dma_idx_r;

uint8_t currentLeapSecond=0;
uint8_t time_synchronization_method=0;
uint8_t time_synchronization_interval=0;

uint8_t downlink_detect_switch=0;
uint16_t downlink_detect_timeout=0;
uint8_t downlink_received_status=0;
uint8_t LoRaMacState_error_times=0;

uint8_t confirmed_uplink_counter_retransmission_increment_switch=0;
uint8_t confirmed_uplink_retransmission_nbtrials=0;

uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=0;
uint8_t LinkADR_NbTrans_retransmission_nbtrials=0;

uint8_t unconfirmed_uplink_change_to_confirmed_uplink_status=0;
uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout=0;

bool payload_oversize;

#define BCD_TO_HEX2(bcd) ((((bcd)>>4)*10)+((bcd)&0x0F))

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            200
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           242
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

uint16_t batteryLevel_mV;
uint8_t payloadlens=0;

extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern uint32_t LoRaMacState;

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

static void StartIWDGRefresh(TxEventType_t EventType);
static void LoraStartRejoin(TxEventType_t EventType);

static void StartCalibrationUTCtime(TxEventType_t EventType);

static void StartDownlinkDetect(TxEventType_t EventType);
static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(TxEventType_t EventType);

TimerEvent_t downlinkLedTimer;
TimerEvent_t NetworkJoinedLedTimer;
TimerEvent_t IWDGRefreshTimer;
TimerEvent_t ReJoinTimer;

TimerEvent_t CalibrationUTCTimer;

TimerEvent_t DownlinkDetectTimeoutTimer;
TimerEvent_t UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer;

uint8_t retrieval_uplink_time=0;
bool is_time_to_reply_downlink=0;
static uint8_t downlink_command_buffer[4];
static uint8_t downlink_command_buffersize=0;

bool joined_led=0,join_led_timeout_status=0;

extern TimerEvent_t TxDelayedTimer;

extern void printf_joinmessage(void);

void OndownlinkLedEvent(void);
void OnNetworkJoinedLedEvent(void);

static void OnIWDGRefreshTimeoutEvent(void);
static void OnReJoinTimerEvent( void );

static void OnCalibrationUTCTimeoutEvent( void );
static void OnDownlinkDetectTimeoutEvent( void );
static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void );

uint8_t HW_GetBatteryLevel( void ); 
uint16_t HW_GetTemperatureLevel( void );	
void HW_GetUniqueId( uint8_t *id );
uint32_t HW_GetRandomSeed( void );
void board_init();
void uart_log_init(uint32_t baudrate);

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/
																	
void uart_log_init(uint32_t baudrate)
{
    lpuart_init_t lpuart_init_cofig;
    uart_config_t uart_config;

    // set iomux
    gpio_set_iomux(GPIOD, GPIO_PIN_12, 2); // LPUART_RX:GP60
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);  // UART0_TX:GP17

    // lpuart init
    lpuart_deinit(LPUART);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPUART, true);
    lpuart_init_cofig.baudrate         = baudrate;
    lpuart_init_cofig.data_width       = LPUART_DATA_8BIT;
    lpuart_init_cofig.parity           = LPUART_PARITY_NONE;
    lpuart_init_cofig.stop_bits        = LPUART_STOP_1BIT;
    lpuart_init_cofig.low_level_wakeup = false;
    lpuart_init_cofig.start_wakeup     = false;
    lpuart_init_cofig.rx_done_wakeup   = true;
    lpuart_init(LPUART, &lpuart_init_cofig);

    lpuart_config_interrupt(LPUART, LPUART_CR1_RX_DONE_INT, ENABLE);
    lpuart_config_tx(LPUART, false);
    lpuart_config_rx(LPUART, true);

    NVIC_SetPriority(LPUART_IRQn, 2);
    NVIC_EnableIRQ(LPUART_IRQn);

    // uart init
    uart_config_init(&uart_config);
    uart_config.fifo_mode = ENABLE;
    uart_config.mode     = UART_MODE_TX;
    uart_config.baudrate = baudrate;
    uart_init(CONFIG_DEBUG_UART, &uart_config);

    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

void board_init()
{
		rcc_enable_oscillator(RCC_OSC_RCO32K, true);
		rcc_set_iwdg_clk_source(RCC_IWDG_CLK_SOURCE_RCO32K);
		rcc_enable_peripheral_clk(RCC_PERIPHERAL_IWDG, true);
    delay_ms(100);	  
    iwdg_init(true);

    iwdg_set_prescaler(IWDG_PRESCALER_256);
    iwdg_set_reload(0xFFF); 
    iwdg_start();	
    rcc_enable_oscillator(RCC_OSC_XO32K, true);

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPUART, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
    #ifdef PRINT_BY_DMA
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA0, true);
    #endif

    delay_ms(100);

	  int rst_src = 0;
	
	  rst_src = RCC->RST_SR;

    if(!(rst_src & 0x24)){
        RtcInit();
			  *((unsigned int *)(0x2000F000)) =0x00;
				*((unsigned int *)(0x2000F001)) =0x00;
				*((unsigned int *)(0x2000F002)) =0x00;
				*((unsigned int *)(0x2000F003)) =0x00;
			
				*((unsigned int *)(0x2000F004)) =0x00;
				*((unsigned int *)(0x2000F005)) =0x00;
			
			  for(int i=0;i<3328;i++)
				{
					*((unsigned int *)(SRAM_SENSOR_DATA_STORE_ACK_START_ADDR+i)) =0xEE;
				}
    }else{
        rtc_calendar_cmd(ENABLE);
        NVIC_EnableIRQ(RTC_IRQn);
        
        RCC->RST_SR |= 0x24;
    }
}

void* _sbrk(int nbytes)
{
    // variables from linker script
    extern char _heap_bottom[];
    extern char _heap_top[];

    static char* heap_ptr = _heap_bottom;

    if ((unsigned int)(heap_ptr + nbytes) <= (unsigned int)_heap_top) {
        void* base = heap_ptr;
        heap_ptr += nbytes;
        return base;
    } else {
        return (void*)(-1);
    }
}

int main(void)
{
    // Target board initialization
    board_init();

	  SX126xInit();
	
    uart_log_init(9600);
	  linkwan_at_init();
	
		BSP_sensor_Init();
		
		StartIWDGRefresh(TX_ON_EVENT);		
		
		/* Configure the Lora Stack*/
		LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
		
	  TimerInit( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer, UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent );
	  TimerInit( &DownlinkDetectTimeoutTimer, OnDownlinkDetectTimeoutEvent );			
    
		#if defined( DRAGINO_LA66 )
		TimerInit( &downlinkLedTimer, OndownlinkLedEvent );
    #endif
		
	  while( 1 )
    {  			
			 if (Radio.IrqProcess != NULL) {
            Radio.IrqProcess();
       }
							
		/* Handle UART commands */
    linkwan_at_process();
		
		if(joined_led==1)
		{
			joined_led=0;
			
			#if defined( DRAGINO_LA66 )
			gpio_write(LED_RGB_PORT, LED_RED_PIN, 0);
			gpio_write(LED_RGB_PORT, LED_GREEN_PIN, 0);
			gpio_write(LED_RGB_PORT, LED_BLUE_PIN, 0);
			
			TimerInit( &NetworkJoinedLedTimer, OnNetworkJoinedLedEvent );
			TimerSetValue( &NetworkJoinedLedTimer, 5000);
			gpio_write(LED_RGB_PORT, LED_GREEN_PIN, 1);
			TimerStart( &NetworkJoinedLedTimer );
      #endif			
		}
		
		if(is_lora_joined==1)
		{
			LOG_PRINTF(LL_DEBUG,"JOINED\n\r");
      rejoin_keep_status=0;
			
			#if defined( ACCUHEALTH_LA66 )
	    gpio_init(LED_RGB_PORT, LED_NET_JOINED_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
			#endif
			
			if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
			{
				printf_joinmessage();
			}			
			
      TimerStop(&ReJoinTimer);
			is_lora_joined=0;
			joined_led=1;
		}
		
    if(join_led_timeout_status==1)	
		{
      join_led_timeout_status=0;			
						
			if(time_synchronization_method==1)
			{
			  StartCalibrationUTCtime(TX_ON_EVENT);
				delay_ms(5);
			}
			
			if(downlink_detect_switch==1)
			{
				if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
				{
					StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(TX_ON_EVENT);
					delay_ms(5);
				}
				StartDownlinkDetect(TX_ON_EVENT);
				delay_ms(5);
			}  		
		}
		
		if(time_synchronization_method==1)
		{
			if(payload_oversize==1)
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					LoRaMacState_error_times=0;
					MlmeReq_t mlmeReq;
					mlmeReq.Type = MLME_DEVICE_TIME;
					LoRaMacMlmeRequest( &mlmeReq );
					payload_oversize=0;
					
					#if defined ( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_AS923 )
					UTC_Request=0;//only send once 
					#endif
					
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 4;
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				}
			}
	  }

		if(atz_flags==1)
		{
			LoRaMacState_error_times=0;
			delay_ms(500);
			AppData.Buff[0]=0x11;
			AppData.BuffSize=1;
			AppData.Port = 4;
			LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			atz_flags++;
		}
		else if((atz_flags==2)&&(( LoRaMacState & 0x00000001 ) != 0x00000001))
		{
			system_reset();
		}
		#ifdef REGION_US915
		if(MAC_COMMAND_ANS_status==1)
		{		
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
          LoRaMacState_error_times=0;          
					MibRequestConfirm_t mib;
			
			    mib.Type=MIB_CHANNELS_DATARATE;
			    LoRaMacMibGetRequestConfirm(&mib);										
					
				  if(mib.Param.ChannelsDatarate==0)
			    {
						MAC_COMMAND_ANS_status=0;
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
	          AppData.Port = 4;
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				  }
				}		  
		}
		#elif defined( REGION_AS923 )	|| defined( REGION_AU915 )		
		if((MAC_COMMAND_ANS_status==1)&&(dwelltime==1))
		{		
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{ 
		    LoRaMacState_error_times=0;
				MibRequestConfirm_t mib;
		
				mib.Type=MIB_CHANNELS_DATARATE;
				LoRaMacMibGetRequestConfirm(&mib);
				
				MAC_COMMAND_ANS_status=0;
				
				if(mib.Param.ChannelsDatarate==2)
				{
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 4;							
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);					
				}
			}		  
		}
		#endif
		
		if((MAC_COMMAND_ANS_status==1 && response_level==3) 
		|| (unconfirmed_downlink_data_ans_status==1 && response_level==1 && is_there_data==1 ) 
		|| (confirmed_downlink_data_ans_status==1 && response_level==2 && is_there_data==1 )
		||(((MAC_COMMAND_ANS_status==1)||(confirmed_downlink_data_ans_status==1&&is_there_data==1))&&(response_level==4)))
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				MAC_COMMAND_ANS_status=0;
				unconfirmed_downlink_data_ans_status=0;
				confirmed_downlink_data_ans_status=0;
				is_there_data=0;
				AppData.Buff[0]=0x00;
				AppData.BuffSize=1;
				AppData.Port = 4;
	      LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			}
		}
		
		if(is_time_to_rejoin==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
			  is_time_to_rejoin=0;
				
				#if defined( ACCUHEALTH_LA66 )
				gpio_init(LED_RGB_PORT, LED_NET_JOINED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
				#endif
				
			  LORA_Join();
			}
		}
		
		if(JoinReq_NbTrails_over==1)
		{
			JoinReq_NbTrails_over=0;
			
			rejoin_keep_status=1;
			
			#if defined( ACCUHEALTH_LA66 )
			gpio_init(LED_RGB_PORT, LED_NET_JOINED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
			#endif
			
			if(REJOIN_TX_DUTYCYCLE==0)
			{
				REJOIN_TX_DUTYCYCLE=20;
			}
			LoraStartRejoin(TX_ON_EVENT);
		}
		
		if(rejoin_status==1 && sleep_status==0)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				rejoin_keep_status=1;
				LoRaMacState_error_times=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
				
				#if defined( ACCUHEALTH_LA66 )
				gpio_init(LED_RGB_PORT, LED_NET_JOINED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
				#endif
				
				TimerStop( &CalibrationUTCTimer);
				TimerStop( &DownlinkDetectTimeoutTimer);
				TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
			  LORA_Join();
			}
		}
	
		if(downlink_received_status==1 && LORA_JoinStatus () == LORA_SET && downlink_detect_switch==1 && downlink_detect_timeout>0 && unconfirmed_uplink_change_to_confirmed_uplink_timeout>0 && sleep_status==0)
		{
			downlink_received_status=0;
			unconfirmed_uplink_change_to_confirmed_uplink_status=0;
			TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
      TimerStart( &DownlinkDetectTimeoutTimer);
			
			if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
			{
				TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
				TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
			}
		}
		
		if(LoRaMacState_error_times>=5 && sleep_status==0)
		{
			LoRaMacState_error_times=0;
			delay_ms(100);
			system_reset();
		}
		
		if(is_time_to_IWDG_Refresh==1)
		{
			is_time_to_IWDG_Refresh=0;
			iwdg_reload();
			
			if(EnterLowPowerStopModeStatus==0)
			{
				EnterLowPowerStopMode_error_times++;
				if(EnterLowPowerStopMode_error_times>=2)
				{
					EnterLowPowerStopMode_error_times=0;
					printf_dma_idx_w=  printf_dma_idx_r;
					printf_dma_busy=0;
				}					
			}			
		}
		
		if( print_isdone( ) ) {
			  EnterLowPowerStopModeStatus=1;
				TimerLowPowerHandler( );
		}
		else
			EnterLowPowerStopModeStatus=0;
  }
}

static void LORA_HasJoined( void )
{	
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );

	is_lora_joined=1;
	
	is_lora_joined_keep_status=1;
}

static void LORA_RxData( lora_AppData_t *AppData )
{
	uint8_t downlink_config_store_in_flash=0;
  is_there_data=1;
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);

	#if defined( DRAGINO_LA66 )
  TimerSetValue(  &downlinkLedTimer, 200);
  gpio_write(LED_RGB_PORT, LED_RED_PIN, 1);
	gpio_write(LED_RGB_PORT, LED_BLUE_PIN, 1);

  TimerStart( &downlinkLedTimer );
	#endif
	
	LOG_PRINTF(LL_DEBUG,"Run AT+RECVB=? to see detail\r\n");			

      switch(AppData->Buff[0] & 0xff)
      {							
        case 4:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]==0xFF)  //---->ATZ
						{
							atz_flags=1;		
						}
						else if(AppData->Buff[1]==0xFE)  //---->AT+FDR
						{	
							uint8_t status[128]={0};
							memset(status, 0x00, 128);

							status[0]=0x12;
							flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
							if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
							{
								LOG_PRINTF(LL_DEBUG,"write config error\r\n");
							}
					
							atz_flags=1;													
						}
					}
					break;
				}	
				
        case 5:
				{
					is_time_to_reply_downlink=1;
					
					if( AppData->BuffSize == 4 )
					{
						if(AppData->Buff[1]<2)
						{
							if(AppData->Buff[1]==0x01)
							{
								lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
								confirmed_uplink_retransmission_nbtrials=AppData->Buff[2];
		            confirmed_uplink_counter_retransmission_increment_switch=AppData->Buff[3];
							}
							else if(AppData->Buff[1]==0x00)
							{
								confirmed_uplink_retransmission_nbtrials=AppData->Buff[2];
		            confirmed_uplink_counter_retransmission_increment_switch=AppData->Buff[3];
								lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
							}
							downlink_config_store_in_flash=1;
					  }
				  }else if(AppData->BuffSize == 2)
					{
            if(AppData->Buff[1]==0x00)
						{
							lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
						}
						else
						{
							lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
						}
						
						confirmed_uplink_counter_retransmission_increment_switch=0;
						confirmed_uplink_retransmission_nbtrials=7;
						downlink_config_store_in_flash=1;
					}else
					{
						is_time_to_reply_downlink=0;
						AppData->BuffSize=0;
					}
					
					downlink_command_buffersize=AppData->BuffSize;
					for(int i=0;i<AppData->BuffSize;i++)
					{
						downlink_command_buffer[i]=AppData->Buff[i];
					}
					
					break;
				}
				
				case 7:
				{
					if( AppData->BuffSize == 2 )
					{
						#if defined( REGION_US915 )	|| defined( REGION_AU915 )
						if(AppData->Buff[1]<9)
						{
							customize_set8channel_set(AppData->Buff[1]);
							downlink_config_store_in_flash=1;
							atz_flags=1;
						}
						#elif defined( REGION_CN470 )
						if(AppData->Buff[1]<13)
						{
							customize_set8channel_set(AppData->Buff[1]);
							downlink_config_store_in_flash=1;
							atz_flags=1;
						}
						#endif
				  }
					break;
				}				
				
				case 0x20:			
				{
					if( AppData->BuffSize == 2 )
					{		
						if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))    
						{
							if(AppData->Buff[1]==0x01)       //---->AT+NJM=1
							{
								lora_config_otaa_set(LORA_ENABLE);
							}
							else                             //---->AT+NJM=0
							{
								lora_config_otaa_set(LORA_DISABLE);							
							}
							downlink_config_store_in_flash=1;
							atz_flags=1;					
						}						 
					}
					break;				
				}	
				
				case 0x21:
				{
					if( (AppData->BuffSize == 2) && (AppData->Buff[1]<=4) )
					{
						response_level=( AppData->Buff[1] );//0~4					//---->AT+RPL
						downlink_config_store_in_flash=1;							
					}
					break;
				}		
				
				case 0x22:			
				{
					MibRequestConfirm_t mib;
					if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))   //---->AT+ADR=1
					{		
						mib.Type = MIB_ADR;
						mib.Param.AdrEnable =AppData->Buff[1];
						LoRaMacMibSetRequestConfirm( &mib );					
						downlink_config_store_in_flash=1;							
					}
					else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x00))   //---->AT+ADR=0
					{
						uint8_t downlink_data_rate=AppData->Buff[2];
						mib.Type = MIB_ADR;					
						mib.Param.AdrEnable = AppData->Buff[1];
						LoRaMacMibSetRequestConfirm( &mib );	
						
						#if defined(REGION_US915)
						if(downlink_data_rate>3)
						{
							downlink_data_rate=3;   
						}
						#elif defined(REGION_AS923) || defined(REGION_AU915)
						if(dwelltime==1)
						{
							if(downlink_data_rate>5)
							{
								downlink_data_rate=5;
							}else if(downlink_data_rate<2)
							{
								downlink_data_rate=2;
							}
						}
						#else
						if(downlink_data_rate>5)
						{
							downlink_data_rate=5;
						}
						#endif	
						
						lora_config_tx_datarate_set(downlink_data_rate) ;
						
						if(AppData->Buff[3]!=0xff)                //---->AT+TXP
						{
							mib.Type = MIB_CHANNELS_TX_POWER;						
							mib.Param.ChannelsTxPower=AppData->Buff[3];
							LoRaMacMibSetRequestConfirm( &mib );							
						}				
						downlink_config_store_in_flash=1;									
					 }
					 break;				
				}								
				
				case 0x25:			
				{
				 #if defined( REGION_AS923 )	|| defined( REGION_AU915 )
				 if( AppData->BuffSize == 2 )
				 {				
					 if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))   //---->AT+DWELLT
					 {
						 dwelltime=AppData->Buff[1];
						 downlink_config_store_in_flash=1;
						 atz_flags=1;		
					 }						
				 }
				 #endif	
				 break;				
				}
				
				case 0x26:
				{
					if( AppData->BuffSize == 3 )
					{
						uint16_t value;			
						
						value=( AppData->Buff[1]<<8 | AppData->Buff[2] );//1~65535
						
						if(value>0)
						{
							REJOIN_TX_DUTYCYCLE=value;
							downlink_config_store_in_flash=1;
						}					
					}
					break;
				}
				
				case 0x27:
				{
					if( AppData->BuffSize == 2 )
					{				
						currentLeapSecond=AppData->Buff[1];
						downlink_config_store_in_flash=1;					
					}
					break;
				}

				case 0x28:
				{
					if( AppData->BuffSize == 2 )
					{				
						time_synchronization_method=AppData->Buff[1];
						if(time_synchronization_method>1)
						{
							time_synchronization_method=1;
						}
						
						if(time_synchronization_method==1)
						{
							StartCalibrationUTCtime(TX_ON_EVENT);
						}
						
						downlink_config_store_in_flash=1;					
					}
					break;
				}
				
				case 0x29:
				{
					if( AppData->BuffSize == 2 )
					{						
						time_synchronization_interval=AppData->Buff[1];
						if(time_synchronization_interval==0)
						{
							time_synchronization_interval=10;
						}
						downlink_config_store_in_flash=1;					
					}
					break;
				}

				case 0x30:
				{
					SysTime_t sysTime = { 0 };
					SysTime_t sysTimeCurrent = { 0 };
					SysTime_t downlinkTime = { 0 };
				
					if( AppData->BuffSize == 6 )
					{				
						downlinkTime.Seconds = ( uint32_t )AppData->Buff[1]<<24;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[2]<<16;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[3]<<8;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[4];
						downlinkTime.SubSeconds = AppData->Buff[5];	
						downlinkTime.SubSeconds = ( int16_t )( ( ( int32_t )downlinkTime.SubSeconds * 1000 ) >> 8 );
						
						sysTime=downlinkTime;

						sysTimeCurrent = SysTimeGet( );
						sysTime = SysTimeAdd( sysTime, SysTimeSub( sysTimeCurrent, LastTxdoneTime ) );

						if(sysTime.Seconds>1611878400)//20210129 00:00:00
						{											
							SysTimeSet( sysTime );
							
							sysTimeCurrent=SysTimeGet();	
							
							UTC_Request=0;
							UTC_Accept=1;
							
							LOG_PRINTF(LL_DEBUG,"Set current timestamp=%u\r",(unsigned int)sysTimeCurrent.Seconds);
						}
						else
						{
							LOG_PRINTF(LL_DEBUG,"timestamp error\r");
						}
					}
					break;
				}				
				
				case 0x32:
				{
					if( AppData->BuffSize == 6 )
					{
            uint16_t value;						
						value=AppData->Buff[1];
						if(value<2)
						{
							downlink_detect_switch=value;
						}
						
            value=AppData->Buff[2]<<8|AppData->Buff[3];
						if(value>0)
						{
							unconfirmed_uplink_change_to_confirmed_uplink_timeout=value;
						}
						
            value=AppData->Buff[4]<<8|AppData->Buff[5];
						if(value>0)
						{
							downlink_detect_timeout=value;
						}
						
						if(downlink_detect_switch==0)
						{
							TimerStop(&DownlinkDetectTimeoutTimer);
							TimerStop(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
						}
						else
						{
							TimerSetValue(&DownlinkDetectTimeoutTimer,downlink_detect_timeout*60000);
							TimerStart(&DownlinkDetectTimeoutTimer);
							
							if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
							{
								TimerSetValue(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
								TimerStart(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
							}	
						}
						
						downlink_config_store_in_flash=1;					
					}
					break;
				}
				
				case 0x33:
				{
					if( AppData->BuffSize == 3 )
					{
						LinkADR_NbTrans_retransmission_nbtrials=AppData->Buff[1];
						LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=AppData->Buff[2];
						
						if(LinkADR_NbTrans_retransmission_nbtrials==0)
						{
							LinkADR_NbTrans_retransmission_nbtrials=1;
						}
						
						if(LinkADR_NbTrans_retransmission_nbtrials>15)
						{
							LinkADR_NbTrans_retransmission_nbtrials=15;
						}
						
						if(LinkADR_NbTrans_uplink_counter_retransmission_increment_switch>1)
						{
							LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=1;
						}
						downlink_config_store_in_flash=1;
					}
					break;
				}				
				
				default:
					break;
			}
	
	if(downlink_config_store_in_flash==1)
	{
		downlink_config_store_in_flash=0;
		Flash_Store_Config();
	}
}

static void OnIWDGRefreshTimeoutEvent( void )
{
	TimerSetValue( &IWDGRefreshTimer,  18000);

  TimerStart( &IWDGRefreshTimer);
	
	if(is_PDTA_command==1 || is_PLDTA_command==1)
	{
		iwdg_reload();
	}
	else
	  is_time_to_IWDG_Refresh=1;
}

static void StartIWDGRefresh(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent );
    TimerSetValue( &IWDGRefreshTimer,  18000); 
		TimerStart( &IWDGRefreshTimer);
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  LOG_PRINTF(LL_DEBUG,"switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  LoRaMacState_error_times=0;
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void OnReJoinTimerEvent( void )
{
	TimerStop( &ReJoinTimer);
	
	is_time_to_rejoin=1;
}

static void LoraStartRejoin(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &ReJoinTimer, OnReJoinTimerEvent );
    TimerSetValue( &ReJoinTimer,  REJOIN_TX_DUTYCYCLE*60000); 
		TimerStart( &ReJoinTimer);
  }
}

static void StartCalibrationUTCtime(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &CalibrationUTCTimer, OnCalibrationUTCTimeoutEvent );
    TimerSetValue( &CalibrationUTCTimer,  time_synchronization_interval*86400000); 
		OnCalibrationUTCTimeoutEvent();
  }
}

static void OnCalibrationUTCTimeoutEvent( void )
{
  TimerSetValue( &CalibrationUTCTimer,  time_synchronization_interval*86400000);
	
  TimerStart( &CalibrationUTCTimer);
	
	UTC_Request=1;
	
	UTC_Accept=0;
	
	if(time_synchronization_method==1)
	{
		#if defined( REGION_US915 )
		MibRequestConfirm_t mib;
		mib.Type=MIB_CHANNELS_DATARATE;
		LoRaMacMibGetRequestConfirm(&mib);
		if(mib.Param.ChannelsDatarate==0)
		{
			payload_oversize=1;//mac command + payload>11
		}
		#elif defined ( REGION_AU915 ) || defined ( REGION_AS923 )
			MibRequestConfirm_t mib;
			mib.Type=MIB_CHANNELS_DATARATE;
			LoRaMacMibGetRequestConfirm(&mib);
			if((mib.Param.ChannelsDatarate==2&&dwelltime==1)||(mib.Param.ChannelsDatarate==0&&dwelltime==0))
			{
				payload_oversize=1;//mac command + payload>11
			}
	  #endif
	}
}

#if defined( DRAGINO_LA66 )
void OndownlinkLedEvent(void)
{
	TimerStop(&downlinkLedTimer);
	gpio_write(LED_RGB_PORT,LED_RED_PIN,0);
	gpio_write(LED_RGB_PORT,LED_BLUE_PIN,0);
}

void OnNetworkJoinedLedEvent(void)
{
	TimerStop(&NetworkJoinedLedTimer);
	gpio_write(LED_RGB_PORT,LED_RED_PIN,0);
	gpio_write(LED_RGB_PORT,LED_GREEN_PIN,0);
	gpio_write(LED_RGB_PORT,LED_BLUE_PIN,0);
}
#endif

static void OnDownlinkDetectTimeoutEvent( void )
{
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		rejoin_status=1;
	}
	
  /*Wait for next tx slot*/
  TimerStop( &DownlinkDetectTimeoutTimer);
}

static void StartDownlinkDetect(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &DownlinkDetectTimeoutTimer, OnDownlinkDetectTimeoutEvent );
    TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
    TimerStart( &DownlinkDetectTimeoutTimer);
  }
}

static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void )
{
	unconfirmed_uplink_change_to_confirmed_uplink_status=1;
	
  /*Wait for next tx slot*/
  TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
}

static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer, UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent );
    TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
    TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
  }
}

uint8_t HW_GetBatteryLevel( void ) 
{
    uint8_t batteryLevel = 0;
	  uint16_t bat_mv=0;
	
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_PCLK1);
	  rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);	

    uint8_t i;
    float gain_value = 1.188f;
    float dco_value = -0.107f;

		uint16_t adc_data[10] = {0};

		float calibrated_sample[10] = {0.0};
		float vbat = 0.0f;

    adc_get_calibration_value(false, &gain_value, &dco_value);

    adc_enable_vbat31(true);

    adc_init();

    delay_us(1000);

    adc_config_clock_division(20); //sample frequence 150K

    adc_config_sample_sequence(0, 15);

    adc_config_conv_mode(ADC_CONV_MODE_CONTINUE);

    adc_enable(true);

	  adc_start(true);
    for (i = 0; i < 10; i++)
    {
        while(!adc_get_interrupt_status(ADC_ISR_EOC));
        adc_data[i] = adc_get_data();
    }

    adc_start(false);
    adc_enable(false);

    for (i = 0; i < 10; i++)
    {//calibration sample value
        calibrated_sample[i] = ((1.2/4096) * adc_data[i] - dco_value) / gain_value;
        
        vbat += calibrated_sample[i] * 3.06;
    }

    vbat /= 10;
		
		bat_mv=vbat*1000;

    if (bat_mv >= 3300)
    {
       batteryLevel = 254;
    }
		else if (bat_mv < 2405)
		{
			batteryLevel = 1;
		}
		else
		{
			batteryLevel = (( (uint32_t) (bat_mv - 2400)*254) /(3300-2400) ); 
		}
		return batteryLevel;
}

uint16_t HW_GetTemperatureLevel( void ) 
{  
  return 0;
}

void HW_GetUniqueId( uint8_t *id )
{
	  uint32_t unique_id[2];
	  system_get_chip_id(unique_id);
	
    id[7] = unique_id[0]>>24&0xFF;
    id[6] = unique_id[0]>>16&0xFF;
    id[5] = unique_id[0]>>8&0xFF;
    id[4] = unique_id[0]&0xFF;
    id[3] = unique_id[1]>>24&0xFF;
    id[2] = unique_id[1]>>16&0xFF;
    id[1] = unique_id[1]>>8&0xFF;
    id[0] = unique_id[1]&0xFF;
}

uint32_t HW_GetRandomSeed( void )
{
		uint32_t unique_id[2];
		system_get_chip_id(unique_id);
		
		return unique_id[0]^unique_id[1];
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
