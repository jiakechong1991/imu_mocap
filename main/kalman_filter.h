#pragma once

//////////////////////卡尔曼
typedef struct {
    float gyro_y;
    float angle;
    float q_bias;
    float angle_err;
    float q_angle;
    float q_gyro;
    float r_angle;
    float dt;
    char c_0;
    float pct_0, pct_1, e;
    float k_0, k_1, t_0, t_1;
    float pdot[4];
    float pp[2][2];
} Kalman;

// 初始化函数，代替C++中的构造函数
void Kalman_Init(Kalman *kalman, float dt);

// 卡尔曼滤波函数
float Kalman_Filter(Kalman *kalman, float accel, float gyro);