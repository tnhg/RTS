/*
*********************************************************************************************************
*                                                 MODULE
*********************************************************************************************************
*/

#ifndef  BSP_H_
#define  BSP_H_


/*
*********************************************************************************************************
*                                     EXTERNAL C LANGUAGE LINKAGE
*
* Note(s) : (1) C++ compilers MUST 'extern'ally declare ALL C function prototypes & variable/object
*               declarations for correct C language linkage.
*********************************************************************************************************
*/

#ifdef __cplusplus
extern  "C" {                                  /* See Note #1.                                         */
#endif

/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
*/

#define abs(x)                  ((x)>0? (x):(-(x)))
#define max(a,b)                ((a)>(b)?(a):(b))
	
/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#define _DEBUG_

#ifdef _DEBUG_
#define INFO(format,...)			printf(format,##__VA_ARGS__)
#else
#define INFO(format,...)
#endif

#define	DBG_USART_EN				0u  // 1: USB  2: Bluetooth  <=0: ITM


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void  BSP_Init (void);

CPU_INT32U  BSP_CPU_ClkFreq (void);

void delay_ms (uint32_t ticks);

/*
*********************************************************************************************************
*                                   EXTERNAL C LANGUAGE LINKAGE END
*********************************************************************************************************
*/

#ifdef __cplusplus
}                                              /* End of 'extern'al C lang linkage.                    */
#endif

/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                         /* End of module include.                               */
