/**
  ******************************************************************************
  * @file    
  * @author  Dragino
  * @version 
  * @date    
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "I2C_A.h"

#define GPIO_PORT_I2C	 GPIOA			
#define I2C_SCL_PIN		 GPIO_PIN_14		
#define I2C_SDA_PIN		 GPIO_PIN_15

#define I2C_SCL_1  gpio_write(GPIO_PORT_I2C, I2C_SCL_PIN, 1)		
#define I2C_SCL_0  gpio_write(GPIO_PORT_I2C, I2C_SCL_PIN, 0)	
	
#define I2C_SDA_1  gpio_write(GPIO_PORT_I2C, I2C_SDA_PIN, 1)		/* SDA = 1 */
#define I2C_SDA_0  gpio_write(GPIO_PORT_I2C, I2C_SDA_PIN, 0)		/* SDA = 0 */
	
#define I2C_SDA_READ()  gpio_read(GPIO_PORT_I2C, I2C_SDA_PIN)	

void I2C_GPIO_MODE_Config(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
	
  gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

uint8_t I2C_Write_Byte(uint8_t addr,uint8_t data)
{
    I2C_Start();
    I2C_SendByte((addr<<1)|0); 
    if(I2C_WaitAck())          
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte(data);        
    if(I2C_WaitAck())         
    {
        I2C_Stop();
        return 1;
    }
    I2C_Stop();
    return 0;
}

uint8_t I2C_Read_Byte(uint8_t addr)
{
    uint8_t res;

	  I2C_Start();                
    I2C_SendByte((addr<<1)|1); 
    I2C_WaitAck();             
    res=I2C_ReadByte(0);		
    I2C_Stop();                 
    return res;  
}

uint8_t I2C_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    I2C_Start();
     I2C_SendByte((addr<<1)|0); 
    if(I2C_WaitAck())          
    {
        I2C_Stop();
        return 1;
    }
//     I2C_SendByte(reg);        
//    I2C_WaitAck();             
    for(i=0;i<len;i++)
    {
         I2C_SendByte(buf[i]); 
        if(I2C_WaitAck())      
        {
            I2C_Stop();
            return 1;
        }
    }
    I2C_Stop();
    return 0;
} 

uint8_t I2C_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
//    I2C_Start();
//     I2C_SendByte((addr<<1)|0); 
//    if(I2C_WaitAck())          
//    {
//        I2C_Stop();
//        return 1;
//    }
//     I2C_SendByte(reg);         
//    I2C_WaitAck();            
	  I2C_Start();                
     I2C_SendByte((addr<<1)|1); 
        if(I2C_WaitAck())      
        {
            I2C_Stop();
            return 1;
        }             
    while(len)
    {
        if(len==1)*buf=I2C_ReadByte(0);
		else *buf=I2C_ReadByte(1);		
		len--;
		buf++;  
    }
    I2C_Stop();                 
    return 0;      
}

void SDA_IN(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_INPUT_FLOATING);
}

void SDA_OUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

void I2C_Delay(void)
{
  volatile int i = 1;
    while (i)

    i--;
}
void I2C_Start(void)
{
	SDA_OUT();	
	I2C_SDA_1;
	I2C_SCL_1;
	I2C_Delay();
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_0;
	I2C_Delay();
}	
void I2C_Stop(void)
{
	SDA_OUT();
	I2C_SCL_0;
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_1;
	I2C_SDA_1;
	I2C_Delay();
}
uint8_t I2C_WaitAck(void)
{
	uint8_t ucErrTime=0;
  SDA_IN();
	I2C_SDA_1;	
	I2C_Delay();
	I2C_SCL_1;	
	I2C_Delay();
	while (I2C_SDA_READ())	
	{
			ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}

	I2C_SCL_0;
	I2C_Delay();
	return 0;
}
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	SDA_OUT();
	I2C_SCL_0;
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)
		{
			I2C_SDA_1;
		}
		else
		{
			I2C_SDA_0;
		}
		I2C_Delay();
		I2C_SCL_1;
		I2C_Delay();	
		I2C_SCL_0;
		I2C_SDA_1; 
		Byte <<= 1;	
		I2C_Delay();
	}
}
uint8_t I2C_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t value;
  SDA_IN();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_0;
		I2C_Delay();
		I2C_SCL_1;
		
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_Delay();
	}
	
	if (!ack)
	{
			I2C_NAck();
	}
	else
	{
			I2C_Ack(); 
	}
		
	return value;
}	

void I2C_Ack(void)
{
	I2C_SCL_0;
	SDA_OUT();
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_1;
	I2C_Delay();
	I2C_SCL_0;
}

void I2C_NAck(void)
{
	I2C_SCL_0;
	SDA_OUT();
	I2C_SDA_1;
	I2C_Delay();
	I2C_SCL_1;
	I2C_Delay();
	I2C_SCL_0;	
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
