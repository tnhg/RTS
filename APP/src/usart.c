/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  USART_MODULE
#include "usart.h"


      
/*
*********************************************************************************************************
*                                  ���ڳ�ʼ��   USART3 �� PD8��PD9 ��
*********************************************************************************************************
*/
void usart_init(void) 
{
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        //GPIOD �� USART3 ʱ��ʹ�ܢ�
        
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); // ʹ�� GPIOD ʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // ʹ�� USART3 ʱ��
        // USART_DeInit(USART3); // ��λ���� 1 ��
        
        GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); // PD9 ����Ϊ USART3
        GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); // PD8 ����Ϊ USART3
        // USART3_TX PD.8 PD.9 ��
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8; // GPIOD9 �� GPIOD8
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ���ù���
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �ٶ� 50MHz
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // ���츴�����
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
        GPIO_Init(GPIOD,&GPIO_InitStructure); // ��ʼ�� PD9��PD8
        
        USART_InitStructure.USART_BaudRate = bound; // һ������Ϊ 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�Ϊ 8 λ���ݸ�ʽ
        USART_InitStructure.USART_StopBits = USART_StopBits_1; // һ��ֹͣλ
        USART_InitStructure.USART_Parity = USART_Parity_No; // ����żУ��λ
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
        USART_Init(USART3, &USART_InitStructure); // ��ʼ������

        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // �����ж�
        // USART3 NVIC���ȼ� ����
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ� 2
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // ��Ӧ���ȼ� 2

        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ ͨ��ʹ��
        NVIC_Init(&NVIC_InitStructure);  // ����ָ���Ĳ�����ʼ�� VIC �Ĵ���
        
        USART_Cmd(USART3, ENABLE); // ʹ�ܴ���
}



void send_uchar(unsigned char udata) // USART3 uchar �����ݷ���
{
  USART_SendData(USART3, udata);
  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
}

void send_char(char data) // USART3 char �����ݷ���
{
  USART_SendData(USART3, data);
  while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) != SET );
}




