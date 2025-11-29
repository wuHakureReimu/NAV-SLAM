

#include <stddef.h>
#include <stdbool.h>
#include "pointcloud.h"



// 上一帧IMU位姿(未修正)，用于计算帧间IMU位姿变换
IMUDataFrame last_IMUDataFrame;

// 上一帧融合位姿
IMUDataFrame last_pose;

// 状态协方差矩阵
double P[6][6] = {0};

// 过程噪声协方差矩阵
double Q[6][6] = {0};

// 观测噪声协方差矩阵
double R[6][6] = {0};

// 卡尔曼增益矩阵
double K[6][6] = {0};

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

    // 观测噪声(我感觉配准不是很准，所以观测噪声调整的比较大)
    for (int i = 0; i < 6; ++i) R[i][i] = 0.3;
}


// 根据IMU数据进行位姿预测
IMUDataFrame predict_pose(IMUDataFrame pose) {
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

// 先验估计协方差矩阵
void predict_cov(IMUDataFrame pose) {
    // 临时调整过程噪声（基于时间间隔，测量时间间隔比较大的话噪声要等比例提高）
    double dt = pose.IMU_timestamps - last_pose.IMU_timestamps;
    if(dt <= 0) dt = 0.1;
    double Q_adjusted[6][6] = {0};
    for(int i = 0; i < 6; i++) Q_adjusted[i][i] = Q[i][i] * dt;
    
    // 更新协方差
    for (int i = 0; i < 6; i++) P[i][i] = P[i][i] + Q[i][i];
}

// 计算卡尔曼增益
void cal_K() {
    for (int i = 0; i < 6; i++) K[i][i] = P[i][i] / (P[i][i] + R[i][i]);
}

// 后验估计位姿状态
IMUDataFrame modify_pose(IMUDataFrame IMU_pose, IMUDataFrame LiDAR_pose) {
    IMUDataFrame result;
    result.IMU_timestamps = IMU_pose.IMU_timestamps;
    result.x = IMU_pose.x + K[0][0] * (LiDAR_pose.x - IMU_pose.x);
    result.y = IMU_pose.y + K[1][1] * (LiDAR_pose.y - IMU_pose.y);
    result.z = IMU_pose.z + K[2][2] * (LiDAR_pose.z - IMU_pose.z);
    result.roll = IMU_pose.roll + K[3][3] * (LiDAR_pose.roll - IMU_pose.roll);
    result.pitch = IMU_pose.pitch + K[4][4] * (LiDAR_pose.pitch - IMU_pose.pitch);
    result.yaw = IMU_pose.yaw + K[5][5] * (LiDAR_pose.yaw - IMU_pose.yaw);
    return result;
}

// 后验估计协方差
void modify_cov() {
    for (int i = 0; i < 6; i++) P[i][i] = (1 - K[i][i]) * P[i][i];
}


// 根据输入的当前帧IMU位姿、当前帧点云配准位姿、上一时刻精确位姿，输出EKF估计的位姿
IMUDataFrame EKF(IMUDataFrame pose, IMUDataFrame lidar_pose) {
    IMUDataFrame current_ekf_pose;  // 存储当前帧融合结果
    
    if (if_first) {
        // 第一帧处理：初始化EKF参数
        init_EKF();
        current_ekf_pose = pose;  // 第一帧无历史数据，用观测值初始化
        last_IMUDataFrame = pose;       // 记录第一帧IMU原始位姿（用于下一帧预积分）
        if_first = false;               // 标记初始化完成
    } else {
        IMUDataFrame predict_pose_result = predict_pose(pose);
        predict_cov(pose);
        cal_K();
        current_ekf_pose = modify_pose(predict_pose_result, lidar_pose);
        modify_cov();
        last_IMUDataFrame = pose;
    }
    
    last_pose = current_ekf_pose;
    return current_ekf_pose;
}