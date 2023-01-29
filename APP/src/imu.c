#include "imu.h"

#define IST8310

/***************************************
   MPU6500解算——几何参数
****************************************/
imu_t imu = {0};
mpu_data_t mpu_data,cal_data={0};
//
//zitai AI = {0};//姿态参数初始化
//float q[8]={0};//姿态中间存储数组
//float g_error[3]={0};//陀螺仪零漂
//
//extern float Sta_Pitch=0;//控制器输出的姿态角
//extern float Sta_Roll=0;

/***************************************
MPU6500解算——控制器参数
****************************************/

 //四元数的元素 ,代表估计方向
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//float exInt=0;
//float eyInt=0;
//float ezInt=0;
//extern double halfT=0.015;
//extern float Ki=0.001;
//extern float Kp=0.8;
//extern uint8_t gyro_cal_sta=0;
//
//uint16_t time_p=0;


///***************************************
//MPU6500解算——陀螺仪标定
//***************************************/
//void error_gyro()
//{
// 
//  for (uint8_t i=1;i<=10;i++)
//  {
//    mpu_data.gx = trans_pi(transmit_16bit_data(MPU6500_GYRO_XOUT_H,MPU6500_GYRO_XOUT_L)*500/32767);//单位为弧度/s
//    mpu_data.gy = trans_pi(transmit_16bit_data(MPU6500_GYRO_YOUT_H,MPU6500_GYRO_YOUT_L)*500/32767);
//    mpu_data.gz = trans_pi(transmit_16bit_data(MPU6500_GYRO_YOUT_H,MPU6500_GYRO_YOUT_L)*500/32767);
//    g_error[0]=g_error[0]+mpu_data.gx;
//    g_error[1]=g_error[1]+mpu_data.gy;
//    g_error[2]=g_error[2]+mpu_data.gz;
//  }
//    g_error[1]=g_error[1]/10.0;
//    g_error[2]=g_error[2]/10.0;
//    g_error[0]=g_error[0]/10.0;
//    
//}
//
//
///***************************************
//MPU6500解算——静态自稳定
//***************************************/
////void stab()
////{
////  get_imu_value();
////  
////  Sta_Pitch=Sta_Pitch-(AI.pitch-AI.p_origin)*pit_Kp_G-AI.gx_fitted*pit_Kd_G;//角度PD控制器
////  Sta_Roll=Sta_Roll-(AI.roll-AI.r_origin)*rol_Kp_G-AI.gy_fitted*rol_Kd_G;
////  
////  //限幅
////  if (Sta_Pitch>=pit_max_ang) 
////    Sta_Pitch=pit_max_ang;
////  else if (Sta_Pitch<=-pit_max_ang) 
////    Sta_Pitch=-pit_max_ang;
////
////  if (Sta_Roll>=rol_max_ang) 
////    Sta_Roll=rol_max_ang;
////  else if (Sta_Roll<=-rol_max_ang) 
////    Sta_Roll=-rol_max_ang;
////  
////  if (gyro_cal_sta==1)
////  {
////    dog.PIT_goal=Sta_Pitch;
////    dog.ROL_goal=Sta_Roll;   
////  }
////  else
////    {
////      Sta_Pitch=0;
////      Sta_Roll=0;
////     }//清空防止跳动抖动
////
////}
//
///***************************************
//MPU6500解算——姿态角pitch、roll、yaw
//****************************************/
////获取处理后的姿态角 存入AI中
////void get_imu_value()
////{
////  
////  
////  get_imu_data();//获得加速度、角速度
////  
////  if (time_p<=99)
////  {
//////      BEEP_Init(593);
////      if (dog.PIT_goal!=0 || dog.ROL_goal!=0)
////      {
////         dog.PIT_goal=0;
////         dog.ROL_goal=0;       
////      }
////      else
////      {
////         AI.p_origin=round(q[0]);
////         AI.r_origin=round(q[1]);
////         time_p=time_p+1;
////         gyro_cal_sta=0;
////      }
////   }
////  else if (time_p==100)
////  {
//////    TIM_Cmd(TIM12, DISABLE);
////    time_p=time_p+1;
////  }
////  else
////  {
////      AI.pitch = q[0];
////      AI.roll = q[1];
////      AI.gx_fitted = mpu_data.gx;  //输出P角速度
////      AI.gy_fitted = mpu_data.gy;  //输出P角速度
////      AI.az_fitted = mpu_data.az;              //输出Z加速度
////      gyro_cal_sta=1;    
////  }
////  
////  IMUupdate(cal_mpu.gx,cal_mpu.gy,cal_mpu.gz,cal_mpu.ax,cal_mpu.ay,cal_mpu.az);
////}
//
////四元数解算+互补滤波
////输入：加速度、角速度
////输出：俯仰角Pitch、翻滚角Roll
//u32 time1 =0;
//u32 time2 =0;
//u32 time3 =0;
//u8 m=0;
//void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az)
//{
////  if(m==0)
////  {
////    time1 = clock();m++;    
////  }
////  else if(m ==1)
////  {
////    time2 = clock();m++;    
////  }
////  else if(m ==2)
////  {
////    time3 = clock();
////    m++;
////  }
//  
//    //float K=0.5;
//    float a[8]={0,0,0,0,0,0,0,0};
//    float vx,vy,vz,ex,ey,ez;
//    float norm,Pitch,Roll;
//    
//    if (ax!=0 || ay!=0 || az!=0) 
//        norm=sqrt(ax*ax+ay*ay+az*az);
//
//    //单位化
//    ax=ax/norm; 
//    ay=ay/norm;
//    az=az/norm;
//    
//    //估计方向的重力
//    vx=2* (q1*q3-q0*q2 );
//    vy=2* (q0*q1+q2*q3 );
//    vz=q0*q0-q1*q1-q2*q2+q3*q3;
//    
//    //错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
//    ex= (ay*vz-az*vy );
//    ey= (az*vx-ax*vz );
//    ez= (ax*vy-ay*vx );
//    
//    //积分误差比例积分增益
//    exInt=exInt+ex*Ki;
//    eyInt=eyInt+ey*Ki;
//    ezInt=ezInt+ez*Ki;
//    
//    //调整后的陀螺仪测量
//    gx=gx+Kp*ex+exInt;
//    gy=gy+Kp*ey+eyInt;
//    gz=gz+Kp*ez+ezInt;
//    
//    //整合四元数率和正常化
//    q0=q0+ (-q1*gx-q2*gy-q3*gz )*halfT;
//    q1=q1+ (q0*gx+q2*gz-q3*gy )*halfT;
//    q2=q2+ (q0*gy-q1*gz+q3*gx )*halfT;
//    q3=q3+ (q0*gz+q1*gy-q2*gx )*halfT;
//    
//    //正常化四元
//    norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3 );
//    q0=q0/norm;
//    q1=q1/norm;
//    q2=q2/norm;
//    q3=q3/norm;
//    Pitch=asin (2*q1*q3-2*q0*q2 )*57.2957795; //pitch ,转换为度数
//    
//    if ((-2*q1*q1-2*q2*q2+1)!=0)
//        Roll=atan2f ((2*q2*q3+2*q0*q1),(-2*q1*q1-2*q2*q2+1) )*57.2957795; ///rollv
////        Roll=atan((2*q2*q3+2*q0*q1)/(q0*q0-q1*q1-q2*q2-q1*q1+q3*q3) )*57.2957795; ///rollv
//    a[0]=Pitch;
//    a[1]=Roll;
////    if ((ay*ay+az*az)!=0)
////        a[2]=-atan(ax/sqrt(ay*ay+az*az))*57.2957795;
////    if ((ax*ax+az*az)!=0)
////        a[3]=atan(ay/sqrt(ax*ax+az*az))*57.2957795;
////    a[4]=gx;
////    a[5]=gy;
////    a[6]=gz;
////    a[0]=-K*Pitch-(1-K)*a[2];
////    a[1]=K*Roll+(1-K)*a[3];
//
//    for(uint8_t i=0;i<=7;i++)
//    {
//      q[i]=a[i];
//    }
//}
//

void imu_ahrs_update(void) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

	now_update  = HAL_GetTick(); //ms
	halfT       = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* Fast inverse square-root */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* compute reference direction of flux */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}


/***************************************
      MPU6500获取值
****************************************/

void get_imu_data()
{
      /*  加速度计数据—+-4g    */
  mpu_data.ax = transmit_16bit_data(MPU6500_ACCEL_XOUT_H,MPU6500_ACCEL_XOUT_L)*4/32767*9.80665;//单位为g 1g=9.8m/s2
  mpu_data.ay = transmit_16bit_data(MPU6500_ACCEL_YOUT_H,MPU6500_ACCEL_YOUT_L)*4/32767*9.80665;
  mpu_data.az = transmit_16bit_data(MPU6500_ACCEL_ZOUT_H,MPU6500_ACCEL_ZOUT_L)*4/32767*9.80665;
        
       /* 陀螺仪数据  +-500dps -> °/s */
  mpu_data.gx = trans_pi(transmit_16bit_data(MPU6500_GYRO_XOUT_H,MPU6500_GYRO_XOUT_L)*500/32767)-g_error[0];//单位为弧度/s,克服零漂
  mpu_data.gy = trans_pi(transmit_16bit_data(MPU6500_GYRO_YOUT_H,MPU6500_GYRO_YOUT_L)*500/32767)-g_error[1];
  mpu_data.gz = trans_pi(transmit_16bit_data(MPU6500_GYRO_YOUT_H,MPU6500_GYRO_YOUT_L)*500/32767)-g_error[2];
        
        //温度数据
  mpu_data.temp = transmit_16bit_data(MPU6500_TEMP_OUT_H,MPU6500_TEMP_OUT_L)/333.87f +21;
        
        //磁力计用于感知东西南北方向
        //加速度计用于感受运动规律
        //陀螺仪用于感受自身姿态变化
  cal_data.ax=mpu_data.ax;
  cal_data.ay=mpu_data.ay;
  cal_data.az=mpu_data.az;
  cal_data.gx=mpu_data.gx;
  cal_data.gy=mpu_data.gy;
  cal_data.gz=mpu_data.gz;
  cal_data.temp=mpu_data.temp;
}

void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = mpu_data.ax;
	hy = mpu_data.ay;
	//hz = mpu_data.az;
	
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
//	
//		if (hx < 0 && hy < 0)
//	{
//		if (fabs(hx / hy) >= 1)
//		{
//			q0 = 0.195;
//			q1 = -0.015;
//			q2 = 0.0043;
//			q3 = 0.979;
//		}
//		else
//		{
//			q0 = 0.555;
//			q1 = -0.015;
//			q2 = 0.006;
//			q3 = 0.829;
//		}
//		
//	}
//	else if (hx < 0 && hy > 0)
//	{
//		if(fabs(hx / hy) >= 1)
//		{
//			q0 = -0.193;
//			q1 = -0.009;
//			q2 = -0.006;
//			q3 = 0.979;
//		}
//		else
//		{
//			q0 = -0.552;
//			q1 = -0.0048;
//			q2 = -0.0115;
//			q3 = 0.8313;
//		}
//		
//	}
//	else if (hx > 0 && hy > 0)
//	{
//		if(fabs(hx / hy) >= 1)
//		{
//			q0 = -0.9785;
//			q1 = 0.008;
//			q2 = -0.02;
//			q3 = 0.195;
//		}
//		else
//		{
//			q0 = -0.9828;
//			q1 = 0.002;
//			q2 = -0.0167;
//			q3 = 0.5557;
//		}
//		
//	}
//	else if (hx > 0 && hy < 0)
//	{
//		if(fabs(hx / hy) >= 1)
//		{
//			q0 = -0.979;
//			q1 = 0.0116;
//			q2 = -0.0167;
//			q3 = -0.195;			
//		}
//		else
//		{
//			q0 = -0.83;
//			q1 = 0.014;
//			q2 = -0.012;
//			q3 = -0.556;
//		}		
//	}
}

//将两个字节合并，转化为字
float transmit_16bit_data(uint8_t addr_h, uint8_t addr_l)
{
    static uint8_t buf[2];
    static short data;
	
  buf[0] = mpu_read_byte(addr_l);
  buf[1] = mpu_read_byte(addr_h);
  data = (buf[1]<<8)|buf[0];
	
  return data;  
}

//角度转弧度
float trans_pi(float x)
{
  float pi = 3.1415926;
  float y;
  y=x*2*pi/360;
  
  return y;
}


/*
*    brief:    initialize imu mpu6500 and magnet meter ist3810
*
*
*/

uint8_t id =0;

void IMU_Init()
{
  OS_ERR err;
  OSTimeDlyHMSM(0u, 0u, 0u, 100u, OS_OPT_TIME_HMSM_STRICT, &err);
  
                             id     =   mpu_read_byte(MPU6500_WHO_AM_I);//获取MPU6500设备id
  
  uint8_t MPU6500_init_data[10][2]   = {
                                        { MPU6500_PWR_MGMT_1, 0x80 },  /*电源管理寄存器，开启电源*/
                                        { MPU6500_PWR_MGMT_1, 0x03 },  /*配置MPU6500时钟源*/
                                        { MPU6500_PWR_MGMT_2, 0x00 },  /*控制传感器失能等*/
                                        
                                        { MPU6500_CONFIG, 0x04 },       /*四个配置寄存器说明如下文所示*/
                                        { MPU6500_GYRO_CONFIG, 0x08 },   /*角速度-+500dps(°/s)*/
                                        { MPU6500_ACCEL_CONFIG, 0x08 },  /*加速度 -+4g*/
                                        { MPU6500_ACCEL_CONFIG_2, 0x02 }, /*开启硬件低通滤波器*/
                                        { MPU6500_SMPLRT_DIV,1},           /*设置采样率500HZ*/
                                        { MPU6500_USER_CTRL, 0x20 },        /*使能从设备AUX通道（如果需要用到从设备则开启）*/
                                      };
  //初始化
  for (uint8_t i = 0; i < 9; i++)
  {
      mpu_write_byte(MPU6500_init_data[i][0], MPU6500_init_data[i][1]);
      OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
  }
  
  mpu_write_byte(MPU6500_GYRO_CONFIG, 3 << 3);
  mpu_write_byte(MPU6500_ACCEL_CONFIG, 2 << 3 );
  
  IST8310_init();
  
}

uint8_t IST8310_init()
{
   OS_ERR err;
  mpu_write_byte(MPU6500_USER_CTRL, 0x30);
   OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
	  /* enable iic 400khz */
  mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
   OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);

    /* turn on slave 1 for ist write and slave 4 to ist read */
  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
   OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
  mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
   OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
  
   ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
   OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);

		/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);

		/* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
  
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    
    return 0;
}

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
   OS_ERR err;
    /* turn off slave 1 at first */
   mpu_write_byte(MPU6500_I2C_SLV1_CTRL,0x00);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
   mpu_write_byte(MPU6500_I2C_SLV1_REG,addr);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
   mpu_write_byte(MPU6500_I2C_SLV1_DO,data);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
    /* turn on slave 1 with one byte transmitting */
   mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    OS_ERR err;
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
    return retval;
}

static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    OS_ERR err;
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    OSTimeDlyHMSM(0u, 0u, 0u, 6u, OS_OPT_TIME_HMSM_STRICT, &err); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    OSTimeDlyHMSM(0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &err);
}



uint8_t mpu_write_byte(uint8_t reg,uint8_t data)
{
  OS_ERR err;
	uint8_t status;
	MPU_NSS_LOW;
	OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
	status = SPI5_RW_Byte(reg); 
	SPI5_RW_Byte(data);
	MPU_NSS_HIGH;
	OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
	return status;
}

 /*
  *
  *   SPI读取寄存器时候，需要在寄存器地址最高位置1*
  */
uint8_t mpu_read_byte(uint8_t reg)
{
   OS_ERR err;
	uint8_t reg_val;        //寄存器值
	MPU_NSS_LOW;            //片选拉低
	OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);         
	SPI5_RW_Byte( reg | 0x80 );     //写入寄存器地址，地址最高位置1，MSB
	reg_val = SPI5_RW_Byte(0xff);  //主机发送一个空字节获取寄存器值
	MPU_NSS_HIGH;     //片选拉高
	OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
	return(reg_val); 
}

/***********************************************************************
*
*                        SPI通信初始化
*
*
*   SPI是双向全双工通信协议，发送和接收是同时进行的。
*   就是说，当单片机用SPI向传感器发送一个字节 同时会收到来自传感器的一个字节数据。
*   
*   所以通常把读写都写在一起，在对传感器寄存器写值时可以丢弃返回的数据，
*   而在读取寄存器值时发送一个空字节，然后接收数据就好了。
*
*
**************************************************************************/

uint8_t SPI5_RW_Byte(uint8_t TxData)
{		 			 
	uint8_t retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET)//如果一直没发送成功
	{
		retry++;
		if(retry > 250)	return 0;
	}			  
	SPI_I2S_SendData(SPI5, TxData);      //发送数据到DR寄存器
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET)//如果一直没接收成功
	{
		retry++;
		if(retry > 250) return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI5);    //从DR寄存器接收数据		    
}

void SPI5_init()  //MPU6500采用SPI进行通信、其上的I2C接口可以接地磁模块形成九轴传感器
{
  	GPIO_InitTypeDef  GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;
  	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI5);
  
	/*  数据总线
         *   SCK  PF7
         *  MISO  PF8
         *  MOSI  PF9
         *
         */
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF,GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9);
	
	/* 片选线（低电平有效）
         *  NSS  PF6
         */
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
        GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF,GPIO_Pin_6);
 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI全双工传输
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;   //设置为SPI主机
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	 //数据位8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	        //10触发极性选择，传输模式
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	      //01
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	//软件片选	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//波特率分频
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  // ！！！MSB先行（高位先行）
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI5, &SPI_InitStructure);
	SPI_Cmd(SPI5, ENABLE);	
}












float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}




//
//
///////////////////方向矩阵的数学运算//////////////////////////////
// 
//unsigned short inv_row_2_scale(const signed char *row)
//{
//    unsigned short b;
// 
//    if (row[0] > 0)
//        b = 0;
//    else if (row[0] < 0)
//        b = 4;
//    else if (row[1] > 0)
//        b = 1;
//    else if (row[1] < 0)
//        b = 5;
//    else if (row[2] > 0)
//        b = 2;
//    else if (row[2] < 0)
//        b = 6;
//    else
//        b = 7;		// error
//    return b;
//}
//
//
///*
//将由0、+1和-1组成的方向矩阵转换为标量表示形式。
//
//转换成标量的方向矩阵。
//
//方向矩阵的描述。最低的2位(0和1)表示该字段所在的列
//
//第一行，以位2为符号。接下来的2位(3和4)表示
//
//第2行第1列，第5位为符号。
//
//接下来的2位(6和7)表示第三行所在的列
//
//第8位是符号。因此，在二进制中，单位矩阵为:
//
//010_001_000或0x88十六进制。
//
//*/
//unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
//{
// 
//    unsigned short scalar;
// 
//    /*
//       XYZ  010_001_000 Identity Matrix
//       XZY  001_010_000
//       YXZ  010_000_001
//       YZX  000_010_001
//       ZXY  001_000_010
//       ZYX  000_001_010
//     */
// 
//    scalar = inv_row_2_scale(mtx);
//    scalar |= inv_row_2_scale(mtx + 3) << 3;
//    scalar |= inv_row_2_scale(mtx + 6) << 6;
// 
// 
//    return scalar;
//}
