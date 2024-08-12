#include "kalman_filter.h"

// 初始化函数，代替C++中的构造函数
void Kalman_Init(Kalman *kalman, float dt) {
    kalman->gyro_y = 0;
    kalman->angle = 0;
    kalman->q_bias = 0;
    kalman->angle_err = 0;
    kalman->q_angle = 0.1f;
    kalman->q_gyro = 0.1f;
    kalman->r_angle = 0.5f;
    kalman->dt = dt; //0.005
    kalman->c_0 = 1;
    kalman->pct_0 = 0;
    kalman->pct_1 = 0;
    kalman->e = 0;
    kalman->k_0 = 0;
    kalman->k_1 = 0;
    kalman->t_0 = 0;
    kalman->t_1 = 0;
    kalman->pdot[0] = 0;
    kalman->pdot[1] = 0;
    kalman->pdot[2] = 0;
    kalman->pdot[3] = 0;
    kalman->pp[0][0] = 1;
    kalman->pp[0][1] = 0;
    kalman->pp[1][0] = 0;
    kalman->pp[1][1] = 1;
}

// 卡尔曼滤波函数
float Kalman_Filter(Kalman *kalman, float accel, float gyro) {
    kalman->angle += (gyro - kalman->q_bias) * kalman->dt;
    kalman->angle_err = accel - kalman->angle;

    kalman->pdot[0] = kalman->q_angle - kalman->pp[0][1] - kalman->pp[1][0];
    kalman->pdot[1] = -kalman->pp[1][1];
    kalman->pdot[2] = -kalman->pp[1][1];
    kalman->pdot[3] = kalman->q_gyro;
    kalman->pp[0][0] += kalman->pdot[0] * kalman->dt;
    kalman->pp[0][1] += kalman->pdot[1] * kalman->dt;
    kalman->pp[1][0] += kalman->pdot[2] * kalman->dt;
    kalman->pp[1][1] += kalman->pdot[3] * kalman->dt;

    kalman->pct_0 = kalman->c_0 * kalman->pp[0][0];
    kalman->pct_1 = kalman->c_0 * kalman->pp[1][0];

    kalman->e = kalman->r_angle + kalman->c_0 * kalman->pct_0;

    kalman->k_0 = kalman->pct_0 / kalman->e;
    kalman->k_1 = kalman->pct_1 / kalman->e;

    kalman->t_0 = kalman->pct_0;
    kalman->t_1 = kalman->c_0 * kalman->pp[0][1];

    kalman->pp[0][0] -= kalman->k_0 * kalman->t_0;
    kalman->pp[0][1] -= kalman->k_0 * kalman->t_1;
    kalman->pp[1][0] -= kalman->k_1 * kalman->t_0;
    kalman->pp[1][1] -= kalman->k_1 * kalman->t_1;

    kalman->angle += kalman->k_0 * kalman->angle_err;
    kalman->q_bias += kalman->k_1 * kalman->angle_err;
    kalman->gyro_y = gyro - kalman->q_bias;
    
    return kalman->angle;
}