/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  LED_MODULE
#include "led.h"

/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* Brief    : 初始化开发板上的所有LED
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void LED1_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  LED_PORT_ENABLE1();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = PIN_LED1;
  GPIO_Init(PORT_LED1, &GPIO_InitStructure);
  GPIO_ResetBits(PORT_LED1, PIN_LED1);
  
  

}

void LED2_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  LED_PORT_ENABLE2();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = PIN_LED2;
  GPIO_Init(PORT_LED2, &GPIO_InitStructure);
  GPIO_ResetBits(PORT_LED2, PIN_LED2);
}

//PG1-PG8 
void LEDG_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  LED_PORT_ENABLEA();
   LED_PORT_ENABLED();
  
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = PIN_LEDG1;
  GPIO_Init(PORT_LEDA, &GPIO_InitStructure);
   GPIO_InitStructure.GPIO_Pin = PIN_LEDG2;
  GPIO_Init(PORT_LEDA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PIN_LEDG3;
  GPIO_Init(PORT_LEDA, &GPIO_InitStructure);
     GPIO_InitStructure.GPIO_Pin = PIN_LEDG4;
  GPIO_Init(PORT_LEDA, &GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin = PIN_LEDG5;
  GPIO_Init(PORT_LEDD, &GPIO_InitStructure);
       GPIO_InitStructure.GPIO_Pin = PIN_LEDG6;
GPIO_Init(PORT_LEDD, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = PIN_LEDG7;
GPIO_Init(PORT_LEDD, &GPIO_InitStructure);
         GPIO_InitStructure.GPIO_Pin = PIN_LEDG8;
  GPIO_Init(PORT_LEDD, &GPIO_InitStructure);
  
  GPIO_SetBits(PORT_LEDA, PIN_LEDG1);
   GPIO_SetBits(PORT_LEDA, PIN_LEDG2);
    GPIO_SetBits(PORT_LEDA, PIN_LEDG3);
     GPIO_SetBits(PORT_LEDA, PIN_LEDG4);
      GPIO_SetBits(PORT_LEDD, PIN_LEDG5);
       GPIO_SetBits(PORT_LEDD, PIN_LEDG6);
        GPIO_SetBits(PORT_LEDD, PIN_LEDG7);
         GPIO_SetBits(PORT_LEDD, PIN_LEDG8);
}

