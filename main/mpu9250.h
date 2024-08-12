#pragma once
#include "esp_log.h"
#include "driver/i2c.h"

////////////////////I2C master配置
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


//////////////////MPU9250寄存器和预定义变量
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

/////////////////////////I2C通信--读写函数
/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 * 从I2c从设备中，在指定寄存器上 读取值
 */
static esp_err_t i2c_device_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 * 从I2C从设备中，在 指定寄存器 上 写入值
 */
static esp_err_t i2c_device_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 * I2C-master初始化
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/////////////////////////////////IMU操作函数///////////////////////////////////////

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
void MPU6050_init() {
    // 电源重新上电
    ESP_ERROR_CHECK(i2c_device_register_write_byte(PWR_MGMT_1, 0x00));
    ESP_ERROR_CHECK(i2c_device_register_write_byte(SMPLRT_DIV, 0x07));
    ESP_ERROR_CHECK(i2c_device_register_write_byte(CONFIG      , 0x07));
    ESP_ERROR_CHECK(i2c_device_register_write_byte(GYRO_CONFIG , 0x18));
    ESP_ERROR_CHECK(i2c_device_register_write_byte(ACCEL_CONFIG, 0x01));
}

float MPU6050_getAccX() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(ACCEL_XOUT_H, r, 2));
    short accx = r[0] << 8 | r[1];
    return (float)accx / AccAxis_Sensitive;
}

float MPU6050_getAccY() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(ACCEL_YOUT_H, r, 2));
    short accy = r[0] << 8 | r[1];
    return (float)accy / AccAxis_Sensitive;
}

float MPU6050_getAccZ() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(ACCEL_ZOUT_H, r, 2));
    short accz = r[0] << 8 | r[1];
    return (float)accz / AccAxis_Sensitive;
}

float MPU6050_getGyroX() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(GYRO_XOUT_H, r, 2));
    short gyrox = r[0] << 8 | r[1];
    return (float)gyrox / GyroAxis_Sensitive;
}

float MPU6050_getGyroY() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(GYRO_YOUT_H, r, 2));
    short gyroy = r[0] << 8 | r[1];
    return (float)gyroy / GyroAxis_Sensitive;
}

float MPU6050_getGyroZ() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(GYRO_ZOUT_H, r, 2));
    short gyroz = r[0] << 8 | r[1];
    return (float)gyroz / GyroAxis_Sensitive;
}

short MPU6050_getTemp() {
    uint8_t r[0];
    ESP_ERROR_CHECK(i2c_device_register_read(TEMP_OUT_H, r, 2));
    return r[0] << 8 | r[1];
}