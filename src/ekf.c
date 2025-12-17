#include "ekf.h"
#include "matrix.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>


// 初始化EKF
void init_ekf(EKF_attr *attr, IMUDataFrame *IMUdata) {
    if (attr == NULL) return;
    
    // 初始化状态
    attr->pos.x = IMUdata->x;
    attr->pos.y = IMUdata->y;
    attr->pos.z = IMUdata->z;
    attr->pos.roll = IMUdata->roll;
    attr->pos.pitch = IMUdata->pitch;
    attr->pos.yaw = IMUdata->yaw;
    
    // 初始化协方差
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            attr->P[i][j] = (i == j) ? 1 : 0.0;    // 初始不确定性可调参
        }
    }
    
    // 初始化过程噪声
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            attr->Q[i][j] = 0.0;
        }
    }
    attr->Q[0][0] = 0.1;    // x 过程噪声
    attr->Q[1][1] = 0.1;    // y 过程噪声
    attr->Q[2][2] = 0.1;    // z 过程噪声
    attr->Q[3][3] = 0.05;   // roll 过程噪声
    attr->Q[4][4] = 0.05;   // pitch 过程噪声
    attr->Q[5][5] = 0.05;   // yaw 过程噪声
    
    // 初始化观测噪声
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            attr->R[i][j] = 0.0;
        }
    }
    attr->R[0][0] = 0.05;   // x 观测噪声
    attr->R[1][1] = 0.05;   // y 观测噪声
    attr->R[2][2] = 0.05;   // z 观测噪声
    attr->R[3][3] = 0.1;   // roll 观测噪声
    attr->R[4][4] = 0.1;   // pitch 观测噪声
    attr->R[5][5] = 0.1;   // yaw 观测噪声
}

// 预测步骤
void ekf_predict(EKF_attr *attr, IMUDataFrame_diff *IMUdata_diff) {
    if (attr == NULL || IMUdata_diff == NULL) return;
    
    // 状态预测：x_k = x_{k-1} + u_k
    attr->pos.x += IMUdata_diff->dx;
    attr->pos.y += IMUdata_diff->dy;
    attr->pos.z += IMUdata_diff->dz;
    attr->pos.roll += IMUdata_diff->droll;
    attr->pos.pitch += IMUdata_diff->dpitch;
    attr->pos.yaw += IMUdata_diff->dyaw;
    
    // 协方差预测：P_k = F * P_{k-1} * F^T + Q
    // 由于状态转移是线性的（x_k = x_{k-1} + u_k），雅可比矩阵F是单位矩阵
    // 所以 P_k = P_{k-1} + Q
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            attr->P[i][j] += attr->Q[i][j];
        }
    }
}

// 修正步骤
void ekf_modify(EKF_attr *attr, Pos *LiDAR_measurement_pos) {
    if (attr == NULL || LiDAR_measurement_pos == NULL) return;
    
    // 测量矩阵H是单位矩阵
    // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
    // 由于H是单位矩阵，简化为：K = P * (P + R)^(-1)
    // 我们先前假定了P和R和Q都是对角矩阵，即六维测量独立，便于计算

    double K[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        K[i][i] = attr->P[i][i] / (attr->P[i][i] + attr->Q[i][i]);
    }
    
    // 计算新息 y = z - H * x = z - x
    double y[6];
    y[0] = LiDAR_measurement_pos->x - attr->pos.x;
    y[1] = LiDAR_measurement_pos->y - attr->pos.y;
    y[2] = LiDAR_measurement_pos->z - attr->pos.z;
    y[3] = LiDAR_measurement_pos->roll - attr->pos.roll;
    y[4] = LiDAR_measurement_pos->pitch - attr->pos.pitch;
    y[5] = LiDAR_measurement_pos->yaw - attr->pos.yaw;
    
    // 状态更新：x = x + K * y
    attr->pos.x += K[0][0] * y[0];
    attr->pos.y += K[1][1] * y[1];
    attr->pos.z += K[2][2] * y[2];
    attr->pos.roll += K[3][3] * y[3];
    attr->pos.pitch += attr->pos.pitch + K[4][4] * y[4];
    attr->pos.yaw += attr->pos.yaw + K[5][5] * y[5];
    
    // 协方差更新：P = (I - K * H) * P = (I - K) * P    
    for (int i = 0; i < 6; i++) {
        attr->P[i][i] = (1 - K[i][i]) * attr->P[i][i];
    }
}