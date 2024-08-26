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
#include <math.h>
#include "driver/i2c.h"
#include "i2c_warp.h"
#include "kalman_filter.h"
#include "wifi_warp.h"
#include "mpu9250.h"


static const char *TAG = "i2c-simple-example";

/// 主函数
void app_main(void)
{
    ESP_LOGI(TAG, "main starting ....");
    // device 初始化
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C-master initialized successfully!!!");
    MPU9250_init(TAG);
	ESP_LOGI(TAG, "imu init success!!!");
    //wifi_init();
    ESP_LOGI(TAG, "wifi init success!!!");

    /////全局变量区
    // imu
    float ax,ay,az,gx,gy,gz,tempure, mx,my,mz;
    float pitch, roll;
    float fpitch, froll;
    Kalman pfilter;
    Kalman rfilter;
    IMUdata imu_data;

    Kalman_Init(&pfilter, 0.005);
    Kalman_Init(&rfilter, 0.005);
    // wifi-udp init
    //SockStr socket_ins = udp_socket_init(TAG);

    uint32_t lasttime = 0;
    int count = 0;

    while(1) {
        // 从IMU中读取原始数据：加速度/角速度/温度
        MPU9250_readAG_MG(&imu_data);
        ax = -imu_data.ACC_X; //x轴加速度
        ay = -imu_data.ACC_Y;
        az = -imu_data.ACC_Z;
        gx = imu_data.Gyro_X; //x轴角速度
        gy = imu_data.Gyro_Y;
        gz = imu_data.Gyro_Z;
        mx = imu_data.Mag_X;
        my = imu_data.Mag_Y;
        mz = imu_data.Mag_Z;
        tempure = imu_data.temputure; //温度


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
            printf("磁力计:(%6.3f,%6.3f,%6.3f)\n", mx, my, mz);
            printf(" Pitch:%6.3f \n", pitch);
            printf(" Roll:%6.3f \n", roll);
            printf(" FPitch:%6.3f \n", fpitch); 
            printf(" FRoll:%6.3f \n", froll);
            //udp_client_send(TAG, &socket_ins);
        }
    }

    // close I2C-master
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
    // close wifi
    // if (socket_ins.sock != -1) {
    //     shutdown(socket_ins.sock, 0);
    //     close(socket_ins.sock);
    //     ESP_LOGE(TAG, "Shutting down socket and restarting...");
    // }

}
