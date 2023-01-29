/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include <includes.h>

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
#define PORT_LED1    GPIOF
#define PORT_LED2    GPIOE
#define PORT_PWR     GPIOH

#define PORT_LEDA   GPIOA
#define PORT_LEDD   GPIOD


#define PIN_LED1    GPIO_Pin_14//бл
#define PIN_LED2    GPIO_Pin_11//╨Л


#define PIN_LEDG1       GPIO_Pin_1
#define PIN_LEDG2       GPIO_Pin_2
#define PIN_LEDG3       GPIO_Pin_3
#define PIN_LEDG4       GPIO_Pin_4
#define PIN_LEDG5       GPIO_Pin_12
#define PIN_LEDG6       GPIO_Pin_13
#define PIN_LEDG7       GPIO_Pin_14
#define PIN_LEDG8       GPIO_Pin_15

#define LED_PORT_ENABLE1()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE ); }
#define LED_PORT_ENABLE2()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE ); }


#define LED_PORT_ENABLEA()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE ); }
#define LED_PORT_ENABLED()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE ); }


#define LED1_Toggle()    GPIO_ToggleBits(PORT_LED1, PIN_LED1)
#define LED1_OFF()        GPIO_SetBits(PORT_LED1, PIN_LED1)
#define LED1_ON()       GPIO_ResetBits(PORT_LED1, PIN_LED1)

#define LED2_Toggle()    GPIO_ToggleBits(PORT_LED2, PIN_LED2)
#define LED2_OFF()        GPIO_SetBits(PORT_LED2, PIN_LED2)
#define LED2_ON()       GPIO_ResetBits(PORT_LED2, PIN_LED2)




#define LEDG1_OFF()        GPIO_SetBits(PORT_LEDA, PIN_LEDG1)
#define LEDG1_ON()       GPIO_ResetBits(PORT_LEDA, PIN_LEDG1)

#define LEDG2_OFF()        GPIO_SetBits(PORT_LEDA, PIN_LEDG2)
#define LEDG2_ON()       GPIO_ResetBits(PORT_LEDA, PIN_LEDG2)

#define LEDG3_OFF()        GPIO_SetBits(PORT_LEDA, PIN_LEDG3)
#define LEDG3_ON()       GPIO_ResetBits(PORT_LEDA, PIN_LEDG3)

#define LEDG4_OFF()        GPIO_SetBits(PORT_LEDA, PIN_LEDG4)
#define LEDG4_ON()       GPIO_ResetBits(PORT_LEDA, PIN_LEDG4)

#define LEDG5_OFF()        GPIO_SetBits(PORT_LEDD, PIN_LEDG5)
#define LEDG5_ON()       GPIO_ResetBits(PORT_LEDD, PIN_LEDG5)

#define LEDG6_OFF()        GPIO_SetBits(PORT_LEDD, PIN_LEDG6)
#define LEDG6_ON()       GPIO_ResetBits(PORT_LEDD, PIN_LEDG6)

#define LEDG7_OFF()        GPIO_SetBits(PORT_LEDD, PIN_LEDG7)
#define LEDG7_ON()       GPIO_ResetBits(PORT_LEDD, PIN_LEDG7)

#define LEDG8_OFF()        GPIO_SetBits(PORT_LEDD, PIN_LEDG8)
#define LEDG8_ON()       GPIO_ResetBits(PORT_LEDD, PIN_LEDG8)


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void LED1_Init (void);
void LED2_Init (void);

void LEDG_Init (void);


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/


