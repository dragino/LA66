 /******************************************************************************
  * @file    bsp.c
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>

#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_adc.h"
#include "bsp.h"
#include "ds18b20.h"
#include "I2C_A.h"
#include "tremo_delay.h"
#include "log.h"
#include "lora_config.h"
#include "flash_eraseprogram.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

#define VBAT_FACTOR     3.06f

void BSP_sensor_Read( sensor_t *sensor_data )
{	

}

void  BSP_sensor_Init( void  )
{
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
	
	#if defined( ACCUHEALTH_LA66 )
	gpio_init(LED_RGB_PORT, LED_NET_JOINED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	gpio_init(LED_RGB_PORT, LED_RECV_DOWNLINK_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	#endif
	
	#if defined( DRAGINO_LA66 )
	gpio_init(LED_RGB_PORT, LED_RED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	gpio_init(LED_RGB_PORT, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	gpio_init(LED_RGB_PORT, LED_BLUE_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	#endif
	
	#if defined( LA66_HARDWARE_TEST )
	gpio_write(LED_RGB_PORT,LED_RED_PIN,1);
	delay_ms(200);
	gpio_write(LED_RGB_PORT,LED_RED_PIN,0);
	gpio_write(LED_RGB_PORT,LED_GREEN_PIN,1);
	delay_ms(200);
	gpio_write(LED_RGB_PORT,LED_GREEN_PIN,0);
	gpio_write(LED_RGB_PORT,LED_BLUE_PIN,1);
	delay_ms(200);
	
	gpio_write(LED_RGB_PORT,LED_RED_PIN,0);
	gpio_write(LED_RGB_PORT,LED_GREEN_PIN,0);
	gpio_write(LED_RGB_PORT,LED_BLUE_PIN,0);
	
	#endif
}

uint16_t battery_voltage_measurement(void)
{
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
        
        vbat += calibrated_sample[i] * VBAT_FACTOR;
    }

    vbat /= 10;
		
		bat_mv=vbat*1000;
		
		if(bat_mv>3700)
		{
			bat_mv=3700;
		}
		
	  return bat_mv;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
