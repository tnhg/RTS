/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define PORT_CAN1						GPIOA
#define PORT_CAN2						GPIOB


#define PIN_CAN1_RX						GPIO_Pin_11
#define PIN_CAN1_TX						GPIO_Pin_12

#define PIN_CAN2_RX						GPIO_Pin_12
#define PIN_CAN2_TX						GPIO_Pin_13


#define CAN1_AF_RX						GPIO_PinSource11
#define CAN1_AF_TX						GPIO_PinSource12
#define CAN1_AF							GPIO_AF_CAN1

#define CAN2_AF_RX						GPIO_PinSource12
#define CAN2_AF_TX						GPIO_PinSource13
#define CAN2_AF							GPIO_AF_CAN2


#define CAN1_PORT_ENABLE()				{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);\
										 RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);}

#define CAN2_PORT_ENABLE()				{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);\
										 RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);\
										 RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);}



/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void CAN1_Init (void);
void CAN2_Init (void);
uint8_t CAN1_Send_Msg (uint16_t CAN_ID, uint8_t *msg);
uint8_t CAN2_Send_Msg (uint16_t CAN_ID, uint8_t *msg);
uint8_t CAN1_TxData (uint16_t CAN_ID, uint8_t *msg, uint8_t len);
 

/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/


