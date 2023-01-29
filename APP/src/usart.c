/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  USART_MODULE
#include "usart.h"


      
/*
*********************************************************************************************************
*                                  串口初始化   USART3 （ PD8，PD9 ）
*********************************************************************************************************
*/
void usart_init(void) 
{
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        //GPIOD 和 USART3 时钟使能①
        
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); // 使能 GPIOD 时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // 使能 USART3 时钟
        // USART_DeInit(USART3); // 复位串口 1 ②
        
        GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); // PD9 复用为 USART3
        GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); // PD8 复用为 USART3
        // USART3_TX PD.8 PD.9 ③
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8; // GPIOD9 与 GPIOD8
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用功能
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度 50MHz
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽复用输出
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
        GPIO_Init(GPIOD,&GPIO_InitStructure); // 初始化 PD9，PD8
        
        USART_InitStructure.USART_BaudRate = bound; // 一般设置为 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为 8 位数据格式
        USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
        USART_Init(USART3, &USART_InitStructure); // 初始化串口

        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启中断
        // USART3 NVIC优先级 配置
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级 2
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // 响应优先级 2

        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ 通道使能
        NVIC_Init(&NVIC_InitStructure);  // 根据指定的参数初始化 VIC 寄存器
        
        USART_Cmd(USART3, ENABLE); // 使能串口
}



void send_uchar(unsigned char udata) // USART3 uchar 型数据发送
{
  USART_SendData(USART3, udata);
  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
}

void send_char(char data) // USART3 char 型数据发送
{
  USART_SendData(USART3, data);
  while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) != SET );
}




