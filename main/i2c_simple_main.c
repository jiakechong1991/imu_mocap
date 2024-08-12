/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "mpu9250.h"
#include "kalman_filter.h"

static const char *TAG = "i2c-simple-example";


/// 主函数
void app_main(void)
{
    ESP_LOGI(TAG, "main starting ....");
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    // 读取imu期间在 I2c注册的地址ID：目前我们是0x68，因为我们的AD0引脚是接GND
    ESP_ERROR_CHECK(i2c_device_register_read(WHO_AM_I, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by reseting the MPU9250 */
    // ESP_ERROR_CHECK(i2c_device_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    ESP_LOGI(TAG, "下面开始初始化imu，并持续数据输出\n");
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    MPU6050_init();
	ESP_LOGI("mpu6050", "init success!");

    float ax,ay,az,gx,gy,gz,tempure;
    float pitch, roll;
    float fpitch, froll;

    Kalman pfilter;
    Kalman rfilter;
    Kalman_Init(&pfilter, 0.005);
    Kalman_Init(&rfilter, 0.005);


    uint32_t lasttime = 0;
    int count = 0;

    while(1) {
        // 从IMU中读取 加速度和角速度和温度
        ax = -MPU6050_getAccX(); //x轴加速度
        ay = -MPU6050_getAccY();
        az = -MPU6050_getAccZ();
        gx = MPU6050_getGyroX(); //x轴角速度
        gy = MPU6050_getGyroY();
        gz = MPU6050_getGyroZ();
        tempure = MPU6050_getTemp();
        // 计算姿态
        pitch = atan(ax/az)*57.2958;  //俯仰角 180/pi=57.2958
        roll = atan(ay/az)*57.2958;  //翻滚角
        fpitch = Kalman_Filter(&pfilter, pitch, gy);
        froll = Kalman_Filter(&rfilter, roll, -gx);
        // 打印输出
        count++;
        if(esp_log_timestamp() / 1000 != lasttime) {
            lasttime = esp_log_timestamp() / 1000;
            count = 0;
            printf("----------------Samples:%d \n", count);
            printf("温度: (%4.2f)\n", tempure);
            printf("加速度:(%4.2f,%4.2f,%4.2f)\n", ax, ay, az);
            printf("陀螺仪:(%6.3f,%6.3f,%6.3f)\n", gx, gy, gz);
            printf(" Pitch:%6.3f \n", pitch);
            printf(" Roll:%6.3f \n", roll);
            printf(" FPitch:%6.3f \n", fpitch);
            printf(" FRoll:%6.3f \n", froll);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    //关闭I2C-master
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
