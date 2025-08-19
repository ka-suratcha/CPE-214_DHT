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

//for boolean type
#include <stdbool.h>

//clock and delay
void SystemClock_Config(void);
void delay40us(void) 
{ 
  uint32_t i = 200;
  while(i > 0) i = i-1;
}

//DHT
void DHT_Set_Pin_Output(void);
void DHT_Set_Pin_Input(void);
int DHT11_read(void);
int status = 0;
uint32_t time_x = 0;
int humidity, temperature;

//EXTI
void EXTI_Init(void);

//button and LCD
bool s = 0;
char disp_str[7] = "START";

//GPIO
void GPIO_Init(void);

int main()
{
	SystemClock_Config();														//max-performance config
  LCD_GLASS_Init();																//LCD Init
  DHT_Set_Pin_Output();														//set DHT's port to output
  GPIO_Init();																		//GPIO Init
	EXTI_Init();																		//EXTI
	
  while(1)
  {		
		//DHT read and get data
		status = DHT11_read();
		LL_mDelay(100);
		
		//check T is over limit?
		if(temperature > 20){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
			DAC->DHR12R2 = 0x0FFF;
		}
		else{ 
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
			DAC->DHR12R2 = 0;
		}
		
		//show data to LCD
		//s from EXTI
		if(s){
			sprintf(disp_str,"H %d", humidity);
		}
		else{ 
			sprintf(disp_str,"T %d", temperature);
		}
		LCD_GLASS_DisplayString((uint8_t*)disp_str);			//diplay on LCD
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

int DHT11_read(void)
{
  uint8_t bits[5] = {0};		//data 8 bits total 40 bits
  uint8_t cnt = 7;					//bit position
  uint8_t idx = 0;					//data's set
  uint8_t i;								//loop

	//DHT START
	LL_mDelay(200);			//clear data
	
  DHT_Set_Pin_Output();														//set DHT's port to output
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);		//pull DHT's port to low
  LL_mDelay(20);																	//delay for 20 ms (at least 18ms)
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);			//set to high
	delay40us();
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);		//pull DHT's port to low
  DHT_Set_Pin_Input();														//set to input, ready to start DHT

	
	//check response DHT
  unsigned int loopCnt = 10000;
  while(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_2) == 0)	//chcek low (wait to go high)
  {
		if(loopCnt-- == 0) return 6; //break;
	}
	
	loopCnt = 30000;
	while(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_2) == 1)	//check high	(wait to go low)
  {
		if(loopCnt-- == 0) return 7; //break;
	}
  
	//read data for DHT
  for(i = 0; i < 40; i++)	//read data 40 bits
  {
		loopCnt = 100000;
		time_x = 0;
    
		//check start transmition
    while(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_2) == 0)	//check low
    {
			if((loopCnt -= 1) == 0) return 8; //break;
		}
    
		//start transmition
    while(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_2) == 1)	//check high
    {
			if((loopCnt -= 1) == 0) 
      {
        return idx; //break;
      }
			else  time_x++;	//look for high
		}
		
		if (time_x > 30){
			bits[idx] |= (1 << cnt);
		}
		
    if (cnt == 0) //shift to last position?
    {
      cnt = 7;   //reset bit position
      idx++;    //next data's set  			
		}
    else  cnt--;  //next position  
  }
  humidity    = (int)(bits[0]); 
  temperature = (int)(bits[2]); 
  
  return 5; //break;
}

void GPIO_Init(void)
{
		LL_GPIO_InitTypeDef GPIO_InitStruct;		         //declare struct for GPIO config

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);	//open clock on AHBENR for GPIOA
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);	//open clock on AHBENR for GPIOB
	
		//PA0 User Button (input)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;							//set port to input
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;										//set on pin 0
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		//set output to push-pull type
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;									//set push-pull to no pull
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;				//set port to fast output speed
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
	
		//PA5 Buzzer (output)
		RCC->APB1ENR |= (1<<29);		//open clock for DAC
		GPIOA->MODER |= (3<<10); 		//alternate function	
		DAC->CR |= (1<<16);					//enable DAC channel 2
	
		//PB11 LED (output)
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;							//set port to input
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;										//set on pin 11
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		//set output to push-pull type
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;									//set push-pull to no pull
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;				//set port to fast output speed
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);									//config to GPIOA register
		
		//PB6 LED on board (output)
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;							//set port to input
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;										//set on pin 6
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		//set output to push-pull type
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;									//set push-pull to no pull
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;				//set port to fast output speed
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);									//config to GPIOB register
}

void EXTI_Init(void)
{
		RCC->APB2ENR |= (1<<0);		//open clock for SYSCFG
	
		SYSCFG->EXTICR[0] &= ~(15<<0);			// EXTI Line 0 (EXTI0) for PA0
		EXTI->RTSR |= (1<<0);								//Rising trigger slection register for PA0
		EXTI->IMR |= (1<<0);								//Interrupt mark for PA0
	
		// NVIC conf
		NVIC_EnableIRQ((IRQn_Type)6);				//use AF 6 (EXTI0)		
		NVIC_SetPriority((IRQn_Type)6,0);		//set priority for PA0 (1st)
}

void EXTI0_IRQHandler(void)
{		//interrupt pressed user button
	if((EXTI->PR & (1<<0)) == 1){
		LL_LCD_Clear();				//clear display
		s = !s;								//Toggle s from display data on LCD
		EXTI->PR |= (1<<0);		//reset EXTI0 for next interrupt
	}
}

void DHT_Set_Pin_Output(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;											//declare struct for GPIO config
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;								//set port to output
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;											//set on pin 2
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;			//set output to push-pull type
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;		        				//set output to push-pull type
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;						//set port to low output speed
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);										//config to GPIOB register
}

void DHT_Set_Pin_Input(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;										//declare struct for GPIO config
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;							//set port to output
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;										//set on pin 2
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;		        			//set output to push-pull type
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;					//set port to low output speed
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);									//config to GPIOB register
}
