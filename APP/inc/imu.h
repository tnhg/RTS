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


#define    MPU_NSS_LOW   GPIO_ResetBits(GPIOF,GPIO_Pin_6) 
#define    MPU_NSS_HIGH  GPIO_SetBits(GPIOF,GPIO_Pin_6) 



typedef struct
{
	float ax;
	float ay;
	float az;

	float mx;
	float my;
	float mz;

	float temp;

	float gx;
	float gy;
	float gz;
	
	float ax_offset;
	float ay_offset;
	float az_offset;

	float gx_offset;
	float gy_offset;
	float gz_offset;
} mpu_data_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float temp;

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;
   
extern mpu_data_t   mpu_data;
extern imu_t        imu;




/***************************************
姿态解算结构体
***************************************/
typedef struct
{
  float pitch;
  float roll;
  float p_origin;
  float r_origin;
  float gx_fitted;
  float gy_fitted;
  float az_fitted;
}posture;

extern posture AI;
extern uint8_t gyro_cal_sta;
extern float q[8];
/***************************************
MPU函数
***************************************/
void SPI5_init();
uint8_t SPI5_RW_Byte(uint8_t TxData);
uint8_t mpu_write_byte(uint8_t reg,uint8_t data);
uint8_t mpu_read_byte(uint8_t reg);

uint8_t IST8310_init();
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data);
static uint8_t ist_reg_read_by_mpu(uint8_t addr);
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num);

void IMU_Init();
void get_imu_data();
void init_quaternion(void);


float transmit_16bit_data(uint8_t addr_h, uint8_t addr_l);
float trans_pi(float x);


//void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az);
//void get_imu_value();
//void stab();
//void error_gyro();

//
//void MPU6500_Port_EXIT_Init(void);
//void MPU6500_I2C_PORT_Init(void);
//void MPU6500_I2C_delay(void);
//void MPU6500_I2C_start(void);
//void MPU6500_I2C_stop(void);
//uint8_t MPU6500_I2C_check_ack(void);
//void MPU6500_I2C_ack(void);
//void MPU6500_I2C_NoAck(void);
//void MPU6500_I2C_write_char(uint8_t dat);
//uint8_t MPU6500_I2C_read_char(void);
//uint8_t MPU6500_write_byte(uint8_t reg,uint8_t data);
//uint8_t MPU6500_read_byte(uint8_t reg);
//uint8_t MPU6500_Read_Len(uint8_t DeviceAddr,uint8_t RegAddr,uint8_t len,uint8_t *pbuff);
//uint8_t MPU6500_Write_Len(uint8_t DeviceAddr,uint8_t RegAddr,uint8_t len,uint8_t *pbuff);
//uint8_t InitMPU6050(void);
//short GetData(uint8_t REG_Address);
//uint8_t MPU6500_Get_Accelerometer(short *ax,short *ay,short *az);
//uint8_t MPU6500_Get_Gyroscope(short *gx,short *gy,short *gz);
//float MPU6500_GetTemperature(void);
//uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr);
//uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr);
//uint8_t MPU6500_Set_Rate(uint16_t rate);
//uint8_t MPU6500_Set_LPF(uint16_t lpf);
//uint8_t MPU6500_run_self_test(void);
//uint8_t MPU6500_DMP_Init(void);
//uint8_t MPU6500_dmp_get_euler_angle(short *accel,short *gyro,float *pitch,float *roll,float *yaw);



/***************************************
MPU6500寄存器
***************************************/
/*此六个寄存器中的值表示在制造测试过程中产生的自测试输出。此值用于检查最终用户执行的后续自测试输出*/
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)

/*此六个寄存器用于消除陀螺仪输出中的直流偏置。在进入传感器寄存器之前，将此寄存器中的值添加到陀螺仪传感器值中。*/
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)

/*此六个寄存器用于消除加速度计输出中的直流偏置。在进入传感器寄存器之前，将此寄存器中的值添加到加速度计传感器值中。*/
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)

/*除以内部采样率(see register CONFIG)生成控制传感器数据输出速率的采样率，FIFO采样率.此寄存器只有在FCHOICE=2‘b11(FCHOICE_B寄存器位为2’b00)和(0<DLPF_CFG<7)时才有效
采样率=内部采样率/(1+SMPLRT_DIV),内部采样率=1 kHz*/
#define MPU6500_SMPLRT_DIV          (0x19)

/*四个配置寄存器说明如下文所示*/
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         GYRO_FS_SEL //   [4:3]   00 - 250dps   01 - 500dps  10 - 1000dps  11 - 2000dps
#define GYRO_FS_SEL                (0x1B)
#define MPU6500_ACCEL_CONFIG        ACCEL_FS_SEL
#define ACCEL_FS_SEL                (0x1C)      //   [4:3]   00 - 2g  01 - 4g  10 - 8g  11 - 16g
#define MPU6500_ACCEL_CONFIG_2      (0x1D)

/*低功率加速度计ODR控制寄存器*/
#define MPU6500_LP_ACCEL_ODR        (0x1E)

/*此寄存器保存x/y/z中断返回值*/
#define MPU6500_MOT_THR             (0x1F)

/*FIFO使能寄存器。若置1，则将对应数据以采样频率写入FIFO*/
#define MPU6500_FIFO_EN             (0x23)

/*IIC主设备控制器，见下文*/
#define MPU6500_I2C_MST_CTRL        (0x24)

/*IIC从设备相关寄存器*/
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)

/*IIC  主设备状态寄存器*/
#define MPU6500_I2C_MST_STATUS      (0x36)
/*三个中断相关寄存器*/
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)

/*这14个寄存器存储加速度、陀螺仪、温度的原始数据*/
#define MPU6500_ACCEL_XOUT_H        (0x3B)   //High byte of accelerometer X-axis data.
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)   //High byte of accelerometer Y-axis data
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)   //High byte of accelerometer Z-axis data.
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)   //High byte of the temperature sensor outpu t    TEMP_degC = ((TEMP_OUT –RoomTemp_Offset)/Temp_Sensitivity) + 21degC
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)    //High byte of the X-axis gyroscope output     GYRO_XOUT = Gyro_Sensitivity * X_angular_rate     Gyro_Sensitivity = 131 LSB/(o/s)
#define MPU6500_GYRO_XOUT_L         (0x44)    
#define MPU6500_GYRO_YOUT_H         (0x45)     //High byte of the Y-axis gyroscope output 
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)      //High byte of the Z-axis gyroscope output 
#define MPU6500_GYRO_ZOUT_L         (0x48)

/*这24个寄存器存储IIC从设备（0、1、2和3)通过辅助IIC接口，从外部传感器读取的数据
从机设备4读取的数据存放在I2C_SLV4_DI中（寄存器53）*/
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)

/*IIC从设备数据输出寄存器*/
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)

#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL          (0x6A)

/*电源管理寄存器，用于配置MPU6500时钟源，控制传感器失能等*/
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)

/*记录写入到FIFO的字节数*/
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)

/*用于从FIFO缓冲区读写数据*/
#define MPU6500_FIFO_R_W            (0x74)

/*存储一个8位数据，用于验证设备的标示*/
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x70
	
#define MPU6500_ID					(0x70)	
#define MPU_IIC_ADDR				0x68


// IST8310 internal reg addr

#define IST8310_ADDRESS         0x0E
#define IST8310_DEVICE_ID_A     0x10

// IST8310 register map. For details see IST8310 datasheet
#define IST8310_WHO_AM_I        0x00
#define IST8310_R_CONFA         0x0A
#define IST8310_R_CONFB         0x0B
#define IST8310_R_MODE          0x02

#define IST8310_R_XL            0x03
#define IST8310_R_XM            0x04
#define IST8310_R_YL            0x05
#define IST8310_R_YM            0x06
#define IST8310_R_ZL            0x07
#define IST8310_R_ZM            0x08

#define IST8310_AVGCNTL          0x41
#define IST8310_PDCNTL          0x42

#define IST8310_ODR_MODE          0x01 //sigle measure mode



float inv_sqrt(float x);

/*    Axis Transformation matrix
|r11  r12  r13| |vx|     |v'x|
|r21  r22  r23| |vy|  =  |v'y|
|r31  r32  r33| |vz|     |v'z|
v'x  = {(r11  * vx) +( r12 * vy) +( r13 * vz)} 
v'y  = {(r21  * vx) +( r22 * vy) +( r23 * vz)}
v'z  = {(r31  * vx) +( r32 * vy) +( r33 * vz)}
*/

unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);

