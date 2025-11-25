

#include <stddef.h>
#include <stdbool.h>
#include "pointcloud.h"



// 上一帧IMU位姿(未修正)，用于计算帧间IMU位姿变换
IMUDataFrame last_IMUDataFrame;

// 状态协方差矩阵
double P[6][6] = {0};

// 过程噪声协方差矩阵
double Q[6][6] = {0};

// 是否是刚开始
bool if_first = true;

// 第一帧：初始化EKF参数
void init_EKF() {
    // 状态协方差
    for (int i = 0; i < 6; ++i) P[i][i] = 1;
    // 位置噪声 (x,y,z)
    Q[0][0] = 0.1; // x噪声方差
    Q[1][1] = 0.1; // y噪声方差  
    Q[2][2] = 0.1; // z噪声方差
    // 姿态噪声 (roll, pitch, yaw)
    Q[3][3] = 0.05; // roll噪声方差
    Q[4][4] = 0.05; // pitch噪声方差
    Q[5][5] = 0.05; // yaw噪声方差
}


// 根据IMU数据进行位姿预测
IMUDataFrame predict_pose(IMUDataFrame pose, IMUDataFrame last_pose) {
    // 检查一下last_pose和last_IMUDataFrame时间戳是否对齐
    if (last_pose.IMU_timestamps != last_IMUDataFrame.IMU_timestamps) return pose;
    IMUDataFrame result;
    // 预测位姿=精确位姿 + Δ(IMU预积分位姿)
    result.pitch = last_pose.pitch + pose.pitch - last_IMUDataFrame.pitch;
    result.roll = last_pose.roll + pose.roll - last_IMUDataFrame.roll;
    result.yaw = last_pose.yaw + pose.yaw - last_IMUDataFrame.yaw;
    result.x = last_pose.x + pose.x - last_IMUDataFrame.x;
    result.y = last_pose.y + pose.y - last_IMUDataFrame.y;
    result.z = last_pose.z + pose.z - last_IMUDataFrame.z;
    return result;
}

// 根据IMU数据进行协方差预测
void predict_covariance(IMUDataFrame pose, IMUDataFrame last_pose) {
    // 状态转移雅可比矩阵 F (6x6)
    // 对于线性运动模型，F通常是单位矩阵
    double F[6][6] = {0};
    for(int i = 0; i < 6; i++) {
        F[i][i] = 1.0;
    }
    
    // 计算帧间时间差用于调整噪声（如果需要）
    double dt = pose.IMU_timestamps - last_pose.IMU_timestamps;
    if(dt <= 0) dt = 0.1; // 默认值
    
    // 临时调整过程噪声（可选，基于时间间隔）
    double Q_adjusted[6][6] = {0};
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            Q_adjusted[i][j] = Q[i][j] * dt; // 根据时间调整噪声
        }
    }
    
    // 协方差预测: P = F * P * F^T + Q
    double F_P[6][6] = {0};
    double F_P_FT[6][6] = {0};
    
    // 计算 F * P
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            for(int k = 0; k < 6; k++) {
                F_P[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    
    // 计算 (F * P) * F^T
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            for(int k = 0; k < 6; k++) {
                F_P_FT[i][j] += F_P[i][k] * F[j][k]; // F^T[j][k] = F[k][j]
            }
        }
    }
    
    // 最终协方差预测: P = F * P * F^T + Q
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            P[i][j] = F_P_FT[i][j] + Q_adjusted[i][j];
        }
    }
}

// 完整的EKF预测步骤
IMUDataFrame predict(IMUDataFrame imu_pose, IMUDataFrame last_estimated_pose) {
    // 状态预测
    IMUDataFrame predicted_pose = predict_pose(imu_pose, last_estimated_pose);
    
    // 协方差预测
    predict_covariance(imu_pose, last_estimated_pose);
    
    return predicted_pose;
}



// 根据输入的当前帧IMU位姿、当前帧点云、上一时刻精确位姿，输出EKF估计的位姿
void EKF(IMUDataFrame pose, LidarDataFrame lidar_frame, IMUDataFrame last_pose) {
    // 根据上一时刻精确位姿和IMU数据，预测当前时刻位姿和协方差
    // TODO
}