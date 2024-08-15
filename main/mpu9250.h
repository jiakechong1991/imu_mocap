#pragma once
#include "esp_log.h"
#include "i2c_warp.h"
#include "freertos/FreeRTOS.h"


//////////////////MPU6050寄存器和预定义变量
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B	
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读) Register addresses of the "who am I" register
#define	MPU6050_ADDR	0x68	//Slave address of the MPU9250 sensor
#define MAG_ADDRESS     0x0C

/////MPU会使用到的常量
#define ORIGINAL_OUTPUT			 (0)
#define ACC_FULLSCALE        	 (2)
#define GYRO_FULLSCALE			 (250)

#if ORIGINAL_OUTPUT == 0
	#if  ACC_FULLSCALE  == 2
		#define AccAxis_Sensitive (float)(16384)
	#elif ACC_FULLSCALE == 4
		#define AccAxis_Sensitive (float)(8192)
	#elif ACC_FULLSCALE == 8
		#define AccAxis_Sensitive (float)(4096)
	#elif ACC_FULLSCALE == 16
		#define AccAxis_Sensitive (float)(2048)
	#endif 
		
	#if   GYRO_FULLSCALE == 250
		#define GyroAxis_Sensitive (float)(131.0)
	#elif GYRO_FULLSCALE == 500
		#define GyroAxis_Sensitive (float)(65.5)
	#elif GYRO_FULLSCALE == 1000
		#define GyroAxis_Sensitive (float)(32.8)
	#elif GYRO_FULLSCALE == 2000
		#define GyroAxis_Sensitive (float)(16.4)
	#endif
		
#else
	#define AccAxis_Sensitive  (1)
	#define GyroAxis_Sensitive (1)
#endif


typedef struct {
    float ACC_X;
    float ACC_Y;
    float ACC_Z;

    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;

    float Mag_X;
    float Mag_Y;
    float Mag_Z;

    float temputure;
} IMUdata;

/*
    MPU6050初始化步骤：
    2. 复位MPU6050： 
        这一步让 MPU6050 内部所有寄存器恢复默认值，通过对电源管理寄存器 1（0X6B）的bit7写 1 实现。
        复位后，电源管理寄存器 1 恢复默认值(0X40)，然后必须设置该寄存器为 0X00，以唤醒 MPU6050，进入正常工作状态。
    3. 设置角速度传感器（陀螺仪）和加速度传感器的满量程范围
        分别通过陀螺仪配置寄存器（0X1B）和加速度传感器配置寄存器（0X1C）设置。
        我们一般设置陀螺仪的满量程范围为±2000dps，加速度传感器的满量程范围为±2g
    4. 设置其他参数：
        我们还需要配置的参数有：关闭中断、关闭 AUX IIC 接口、禁止 FIFO、设置陀螺仪采样率和设置数字低通滤波器（DLPF）等
    5. 配置系统时钟源：
        系统时钟源同样是通过电源管理寄存器 1（0X6B）来设置，该寄存器的最低三位用于设置系统时钟源选择，默认值是 0（内部 8M RC 震荡），
        不过我们一般设置为 1，选择 x 轴陀螺 PLL作为时钟源，以获得更高精度的时钟。
    6. 使能角速度传感器和加速度传感器：
        这两个操作通过电源管理寄存器 2（0X6C）来设置，设置对应位为 0 即可开启。
*/
void MPU9250_init(char *TAG) {
    
    // 读取imu期间在 I2c注册的地址ID：目前我们是0x68，因为我们的AD0引脚是接GND
    uint8_t data[2];
    // 读取who_am_i寄存器
    ESP_ERROR_CHECK(i2c_device_register_read(MPU6050_ADDR, WHO_AM_I, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    // 电源重新对imu上电
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU6050_ADDR, PWR_MGMT_1, 0x00));
    //vTaskDelay(100);
    // 设置陀螺仪采样率
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU6050_ADDR, SMPLRT_DIV, 0x07));
    // 设置低通滤波频率
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU6050_ADDR, CONFIG      , 0x07));
    // 陀螺仪测量范围
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU6050_ADDR, GYRO_CONFIG , 0x18));
    // 加速度技测量范围
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU6050_ADDR, ACCEL_CONFIG, 0x01));
}


void MPU9250_readAG_MG(IMUdata *imu_data)  //读取加速度/陀螺仪/磁力记 的数据
{
    uint8_t Buf[14];
    ESP_ERROR_CHECK(i2c_device_register_read(MPU6050_ADDR, 0x3B, Buf, 14));

    // 加速度
    short accx=(Buf[0]<<8 | Buf[1]);
    imu_data->ACC_X = (float)accx / AccAxis_Sensitive;
    short accy=(Buf[2]<<8 | Buf[3]);
    imu_data->ACC_Y = (float)accy / AccAxis_Sensitive;
    short accz=Buf[4]<<8 | Buf[5];
    imu_data->ACC_Z = (float)accz / AccAxis_Sensitive;
    
    //temputure
    imu_data->temputure = (float) (Buf[6] << 8 | Buf[7]);

    //陀螺仪
    short gyrox=(Buf[8]<<8 | Buf[9]);
    imu_data->Gyro_X = (float)gyrox / GyroAxis_Sensitive;
    short gyroy=(Buf[10]<<8 | Buf[11]);
    imu_data->Gyro_Y = (float)gyroy / GyroAxis_Sensitive;
    short gyroz=(Buf[12]<<8 | Buf[13]);
    imu_data->Gyro_Z = (float)gyroz / GyroAxis_Sensitive;
    
    uint8_t rawData[6];  // x/y/z gyro calibration data stored here
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU6050_ADDR, 0x37, 0x02)); // Power down magnetometer  
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MAG_ADDRESS, 0x0A, 0x01)); // Enter Fuse ROM access mode
    ESP_ERROR_CHECK(i2c_device_register_read(MAG_ADDRESS, 0x10, rawData, 3));
    imu_data->Mag_X = (rawData[1]<<8 | rawData[0]);
    imu_data->Mag_Y = (rawData[3]<<8 | rawData[2]);
    imu_data->Mag_Z = (rawData[5]<<8 | rawData[4]);
    ESP_ERROR_CHECK(i2c_device_register_write_byte(MAG_ADDRESS, 0x0A, 0x00)); // Power down magnetometer  

}


