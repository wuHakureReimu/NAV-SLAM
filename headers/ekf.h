
#ifndef EKF_H
#define EKF_H

#include <stdbool.h>
#include "pointcloud.h"


// IMU 差分数据
typedef struct
{
    int timestamps_diff;
    double dx, dy, dz, droll, dpitch, dyaw;
} IMUDataFrame_diff;

// EKF算法维护的属性
typedef struct
{
    Pos pos;            // 估计位姿
    double P[6][6];     // 估计协方差

    double Q[6][6];     // 过程噪声（目前是常数）
    double R[6][6];     // 观测噪声（目前是常数，可以根据配准残差进行调整）
} EKF_attr;

// 用第一帧数据初始化
void init_ekf(EKF_attr *attr, IMUDataFrame *IMUdata);

// 模型预测
void ekf_predict(EKF_attr *attr, IMUDataFrame_diff *IMUdata_diff);

// 观测修正
void ekf_modify(EKF_attr *attr, Pos *LiDAR_measurement_pos);

#endif