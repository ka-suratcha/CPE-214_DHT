//Based registers included
#include "stm32l1xx.h"

//Base LL driver included
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_bus.h"

//LL driver for LCD included
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"

//for sprinf function access
#include <stdio.h> 

//for bool type
#include <stdbool.h>

//base setup
void SystemClock_Config(void);		//max-performance config
void GPIO_Init(void);							//config GPIO

//delay
void uDelay(uint32_t);

//GPIO DHT11 Config
void DHT_Set_Pin_Output (void);		//set DHT's port to output
void DHT_Set_Pin_Input (void);		//set DHT's port to input

//DHT11 type
struct DHT_DataTypedef 
{
	float Temperature;
	float Humidity;
};

//DHT11 procress
void DHT_start(void);													//start DHT
uint8_t DHT_Check_Response(void);							//check DHT
uint8_t DHT_Read(void);												//read DHT
void DHT_GetData(struct DHT_DataTypedef *);		//getchar data for DHT

struct DHT_DataTypedef DHT11_Data;		
float T = 0;		//Temperature
float H = 0;		//Humidity
uint8_t H_byte1, H_byte2, T_byte1, T_byte2;
uint16_t SUM; uint8_t Presence = 0;

//user button
bool status = 0;
uint8_t usr_button;

//LCD
char disp_str[7];

int main() {
	SystemClock_Config();		//max-performance config
	GPIO_Init();						//call function for GPIO Init
	LCD_GLASS_Init();				//call function for LCD Init	
	
	
	//DAC setup
	RCC ->APB1ENR |= (1<<29);		//open clock for DAC
	DAC->CR |= (1<<16);					//enable DAC channel 2
	
	while(1){
		//DHT11
		DHT_GetData(&DHT11_Data);			//run DHT11
		T = DHT11_Data.Temperature;		//getc T
	  H = DHT11_Data.Humidity;			//get H
		LL_mDelay(3000);							//delay 3s
		
		//User button
		usr_button = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
		if (usr_button == 1 && !status) {
			sprintf(disp_str,"T %.2f/n", T);
			LCD_GLASS_DisplayString((uint8_t*)disp_str);		//diplay on LCD
			LL_mDelay(1000);																//set delay
			status = 1; 																		//set status to true
		}
		else if (usr_button == 1 && status) {
			sprintf(disp_str,"H %.2f/n", H);
			LCD_GLASS_DisplayString((uint8_t*)disp_str);		//diplay on LCD
			LL_mDelay(1000);																//set delay
			status = 0; 																		//set status to false
		}
		
		//Buzzer
		if(T > 40){
			DAC->DHR12R1 = 0x0FFF;  												//Vin = 0FFFh = 4095d
		}
		
	}
}	

void GPIO_Init(void)
{	//setup GPIO
		LL_GPIO_InitTypeDef GPIO_InitStruct;		//declare struct for GPIO config

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);		//open clock on AHBENR for GPIOA
	
		//PA0 User Button (input)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;							//set port to input
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;										//set on pin 0
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		//set output to push-pull type
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;									//set push-pull to no pull
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;				//set port to fast output speed
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
	
		//PA4 DHT11 (input)
		GPIO_InitStruct.Pin = LL_GPIO_PIN_4;										//set on pin 0
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;					//set port to low output speed
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
	
		//PA5 Buzzer (output)
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;							//set port to output
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;										//set on pin 0
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;				//set port to fast output speed
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
}

void DHT_Set_Pin_Output (void)
{	//set DHT's port to output
	LL_GPIO_InitTypeDef GPIO_InitStruct;										//declare struct for GPIO config
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;							//set port to output
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;										//set on pin 4
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		//set output to push-pull type
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;					//set port to low output speed
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
}

void DHT_Set_Pin_Input (void)
{	//set DHT's port to output
	LL_GPIO_InitTypeDef GPIO_InitStruct;										//declare struct for GPIO config
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;							//set port to input
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;										//set on pin 4
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;									//set push-pull to no pull
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
}

void DHT_Start(void)
{	//start DHT
	
	//MCU send start signal, pull down for 18ms
	DHT_Set_Pin_Output();														//set the pin DHT11 as output
	
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);		//pull the pin low
//	LL_mDelay(6); 																	//wait for 18ms (21ms)
	
	//MCU pulls up for 20-40us
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);		//pull the pin high
//	uDelay(45); 																	//wait for 20-40us (21us)
	
	//ready for DHT response
	DHT_Set_Pin_Input();		//set as input

}

uint8_t DHT_Read (void)
{	//read data from DHT
	uint8_t i,j;
	for (j = 0; j < 8; j++)
	{
		while (!(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)));   // wait for the pin to go high
		uDelay(60);   // wait for 40 us (43.3us)
		
		if (!(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)))   // if the pin is low
		{
			i &= ~(1<<(7-j));   // write 0
		}
		else {
			i |= (1<<(7-j));  // if the pin is high, write 1
			while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4));  // wait for the pin to go low
		}
	}
	return i;
}

void DHT_GetData (struct DHT_DataTypedef *DHT_Data)
{	//getc data from DHT
  DHT_Start();												//run DHT11
	
	Presence = DHT_Check_Response();		//check DHT11 responese
	
	H_byte1 = DHT_Read();								//8 bit inregral RH data
	H_byte2 = DHT_Read();								//8 bit decimal RH data
	T_byte1 = DHT_Read();								//8 bit integral T data
	T_byte2 = DHT_Read();								//8 bit decimal T data
	SUM = DHT_Read();										//8 bit  checksum

	if (SUM == (H_byte1+H_byte2+T_byte1+T_byte2)) {		//check data
			DHT_Data->Temperature = T_byte1;
			DHT_Data->Humidity = H_byte1;
	}
}

void uDelay(uint32_t i)
{	//delay in 2 us per unit
	while(i != 0){
	i -= 1;
	}
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}

uint8_t DHT_Check_Response(void)
{ // check DHT response
	//	uint16_t i = 0;
	// DHT sends out response signal and wait for 80ms
	//	for(i = 5; i > 0; )
	//	{
	//		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)) uDelay(1);	//wait to go low
	//		else	 break;
	//		if(--i == 0) return 0;
	//  }
	while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
		; // wait to go low
	//	i = 10000;
	//	while(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)){
	//		if(i-- == 0) break;
	//	}

	//	for(i = 40; i > 0; )
	//	{
	//		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)) break;			//wait to go high
	//		else	uDelay(1);
	//		if(--i == 0) return 1;
	//  }
	while (!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
		; // wait to go high

	//	i = 60000;
	//	while(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)){
	//		if(i-- == 0) break;
	//	}
	//

	//	for(i = 40; i > 0; )
	//	{
	//		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)) uDelay(1);	//wait to go low
	//		else						break;
	//		if(--i == 0) return 2;
	//  }
	while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
		; // wait to go low

	return 3;
}