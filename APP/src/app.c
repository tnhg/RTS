//*********************************************************************************************************    ͷ�ļ�

#define   APP_MODULE
#include  <includes.h> //�������е�ͷ�ļ��Լ����ܺ�����ʼ��������һ�ַ�װ��ʽ


//#define PREPARE_OK  1
//*********************************************************************************************************    �������

//OS_TCB ��һ����������ĸ������ʵĽṹ�壬������os.h�ļ���

//����һ��**������ƿ�**�Ĵ�������������Ҳ��һ����������ϵͳ���Ǳز����ٵ�
static  OS_TCB   TaskStartTCB;         //����������ƿ�   
static  CPU_STK  TaskStartStk[TASK_START_STK_SIZE]; //���������ջ�Ĵ�С��һ����˵��С����һ����

//����һ���������Ե���������
static  OS_TCB   TaskTESTTCB;
static  CPU_STK  TaskTESTStk[TASK_TEST_STK_SIZE];


OS_FLAG_GRP InitStatus;

//*********************************************************************************************************    ����ģ��

//����ģ��
static  void	TaskStart    		(void *p_arg); 
static  void	TaskTEST    	        (void *p_arg);



/*********************************************************************************************************     ������
 ������
*
* ˵������Ҫ������������Ĵ����Լ�һ�������ѭ�����ڴ˿�ʼ������Ĵ���
*
*
*********************************************************************************************************/   

int main()
{
  OS_ERR err;
  
  CPU_IntDis(); // ���������ж�
  
  OSInit(&err); // ��ʼ��Uc/os-III
  
  OSTaskCreate(&TaskStartTCB,     // ������ʼ����
               "Task Start",     //��������
               TaskStart,        //������
               0u,               //���ݸ��������Ĳ���
               TASK_START_PRIO,    //�������ȼ�
               &TaskStartStk[0u],    //�����ջ����ַ
               (TASK_START_STK_SIZE / 10u),    //�����ջ�����λ
               TASK_START_STK_SIZE,     //�����ջ��С
               0u,       //�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ
               0u,        //ʹ��ʱ��Ƭ�ֵ��ȵ�ʱ��Ƭ����
               0u,      //����洢��
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),  //����ѡ��
               &err);     //��Ÿú�������ʱ�ķ���ֵ
  
  OSStart(&err); // ��ʼ�������� 
  
  while (DEF_ON)
  { 
    //���򲻻�ִ�е��� while ѭ��
  }
}

/**********************************************************************************************************    ��������    
��������

* ˵��       :    ����һ������������������Ӧ������������Ĵ���
*
* ����       :    p_arg�ǡ�ostaskCreate���������ݸ���apptaskStart�������Ĳ���
*
* ����ֵ     :    �޷���ֵ
*
* ע��       :    ��һ�д������ڷ�ֹ���������棬��Ϊδʹ�á�p_arg������������ӦΪ����������κδ���
*********************************************************************************************************/

static void TaskStart (void *p_arg)
{
  OS_ERR err;
  (void)p_arg;
  
  CPU_Init(); // ��ʼ��UC/CPU����
  Mem_Init(); // ��ʼ���ڴ����ģ��
  Math_Init();
  CPU_IntEn(); // ���������ж�
  BSP_Init(); 

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err); // ��û���������е�����¼���CPU����
#endif
  
#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
    
    
  OSFlagCreate(&InitStatus,   //����һ���¼������
               "Init Status",
               (OS_FLAGS)0,
               &err);
  
  OSTaskCreate(&TaskTESTTCB,   //��������LED������                          		
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
     // �����壬ʼ����Ϊ����ѭ��д��
    
    OSTimeDlyHMSM(0u,  0u,  0u, 300u, OS_OPT_TIME_HMSM_STRICT, &err); // ϵͳ��ʱ
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
    ////////////////PWM���//////////////////////////////////////
//    TIM_SetCompare1(TIM2,2000);
    //////////////���ڷ���/////////////////////////////////////////
//    send_uchar(0x01);
//    send_char(0x01);
    ////////////////��ˮ��///////////////////////////////////////
     LEDG1_ON();   
     LEDG2_ON();   
     LEDG3_ON();    
     LEDG4_ON();
     OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err);
     LEDG5_OFF();   
     LEDG6_OFF();  
     LEDG7_OFF();
     LEDG8_OFF();
    
    ///////////////////��ˮ��////////////////////////////////////////////
  }
}

