#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// 定义Lidar数据帧结构
#define MAX_ROWS 8
#define MAX_COLS 8


//雷达帧
typedef struct {
    int ToF_timestamps;  // 时间戳
    int ToF_distances[MAX_ROWS][MAX_COLS];  // 距离矩阵
} LidarDataFrame;

// 定义IMU数据帧结构
typedef struct {
    int IMU_timestamps;  // 时间戳
    double x;
    double y;
    double z;
} IMUDataFrame;

// 定义 Point 结构体，用于表示3D坐标
typedef struct {
    double x;  // x 坐标
    double y;  // y 坐标
    double z;  // z 坐标
} Point;

// 点云帧
typedef struct {
    int ToF_timestamps;        // 时间戳
    Point ToF_position [MAX_ROWS][MAX_COLS];  // 3D点矩阵，每个元素是一个 Point
} PointCloud;


// 位姿信息结构体
typedef struct {
    double imu_x;      // IMU原始x坐标 (mm)
    double imu_y;      // IMU原始y坐标 (mm)
    double imu_z;      // IMU原始z坐标 (mm)
    double modified_x; // mapping修正后的x坐标 (mm)
    double modified_y; // mapping修正后的y坐标 (mm)
    double modified_z; // mapping修正后的z坐标 (mm)
} PoseInfo;

#endif