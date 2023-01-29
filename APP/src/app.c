//*********************************************************************************************************    头文件

#define   APP_MODULE
#include  <includes.h> //包含所有的头文件以及功能函数初始化，这是一种封装方式


//#define PREPARE_OK  1
//*********************************************************************************************************    任务变量

//OS_TCB 是一个包含任务的各种性质的结构体，定义在os.h文件中

//这是一个**任务控制块**的创建，而它本身也是一个任务，这在系统中是必不可少的
static  OS_TCB   TaskStartTCB;         //定义任务控制块   
static  CPU_STK  TaskStartStk[TASK_START_STK_SIZE]; //定义任务堆栈的大小，一般来说大小都是一样的

//这是一个用来测试的任务例程
static  OS_TCB   TaskTESTTCB;
static  CPU_STK  TaskTESTStk[TASK_TEST_STK_SIZE];


OS_FLAG_GRP InitStatus;

//*********************************************************************************************************    函数模型

//函数模型
static  void	TaskStart    		(void *p_arg); 
static  void	TaskTEST    	        (void *p_arg);



/*********************************************************************************************************     主函数
 主函数
*
* 说明：主要包含启动任务的创建以及一个大的死循环，在此开始多任务的处理
*
*
*********************************************************************************************************/   

int main()
{
  OS_ERR err;
  
  CPU_IntDis(); // 禁用所有中断
  
  OSInit(&err); // 初始化Uc/os-III
  
  OSTaskCreate(&TaskStartTCB,     // 创建开始任务
               "Task Start",     //任务名称
               TaskStart,        //任务函数
               0u,               //传递给任务函数的参数
               TASK_START_PRIO,    //任务优先级
               &TaskStartStk[0u],    //任务堆栈基地址
               (TASK_START_STK_SIZE / 10u),    //任务堆栈深度限位
               TASK_START_STK_SIZE,     //任务堆栈大小
               0u,       //任务内部消息队列能够接收的最大消息数目
               0u,        //使能时间片轮调度的时间片长度
               0u,      //补充存储区
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),  //任务选项
               &err);     //存放该函数错误时的返回值
  
  OSStart(&err); // 开始多任务处理 
  
  while (DEF_ON)
  { 
    //程序不会执行到此 while 循环
  }
}

/**********************************************************************************************************    启动任务    
启动任务

* 说明       :    这是一个启动任务函数，这里应包含其他任务的创建
*
* 参数       :    p_arg是“ostaskCreate（）”传递给“apptaskStart（）”的参数
*
* 返回值     :    无返回值
*
* 注释       :    第一行代码用于防止编译器警告，因为未使用“p_arg”，编译器不应为此语句生成任何代码
*********************************************************************************************************/

static void TaskStart (void *p_arg)
{
  OS_ERR err;
  (void)p_arg;
  
  CPU_Init(); // 初始化UC/CPU服务
  Mem_Init(); // 初始化内存管理模块
  Math_Init();
  CPU_IntEn(); // 启用所有中断
  BSP_Init(); 

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err); // 在没有任务运行的情况下计算CPU容量
#endif
  
#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
    
    
  OSFlagCreate(&InitStatus,   //创建一个事件标记组
               "Init Status",
               (OS_FLAGS)0,
               &err);
  
  OSTaskCreate(&TaskTESTTCB,   //创建点亮LED的任务                          		
               "TEST",
               TaskTEST,
               0u,
               TASK_TEST_PRIO,
               &TaskTESTStk[0u],
               (TASK_TEST_STK_SIZE / 10u),
               TASK_TEST_STK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &err);
  
  
    INFO("Start Task\n");
  while (DEF_TRUE) 
  { 
     // 任务体，始终作为无限循环写入
    
    OSTimeDlyHMSM(0u,  0u,  0u, 300u, OS_OPT_TIME_HMSM_STRICT, &err); // 系统延时
    //             h   min   s   ms
  }
}



static void TaskTEST (void *p_arg) 
{
  OS_ERR err;
  //CPU_TS ts;
  (void)p_arg;
  //OSFlagPend(&InitStatus, PREPARE_OK, 0, OS_OPT_PEND_FLAG_SET_ALL + OS_OPT_PEND_BLOCKING, &ts, &err);
  OSTimeDlyHMSM(0u, 0u, 0u, 200u, OS_OPT_TIME_HMSM_STRICT, &err);
  
  while (DEF_TRUE) 
  {
    ////////////////PWM输出//////////////////////////////////////
//    TIM_SetCompare1(TIM2,2000);
    //////////////串口发送/////////////////////////////////////////
//    send_uchar(0x01);
//    send_char(0x01);
    ////////////////流水灯///////////////////////////////////////
     LEDG1_ON();   
     LEDG2_ON();   
     LEDG3_ON();    
     LEDG4_ON();
     OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err);
     LEDG5_OFF();   
     LEDG6_OFF();  
     LEDG7_OFF();
     LEDG8_OFF();
    
    ///////////////////流水灯////////////////////////////////////////////
  }
}

