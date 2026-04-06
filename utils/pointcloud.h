#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// 定义Lidar数据帧结构
#define L5_MAX_ROWS 8
#define L5_MAX_COLS 8
#define L9_MAX_ROWS 54
#define L9_MAX_COLS 42
#define MAX_ROWS L5_MAX_ROWS // 在此处配置雷达分辨率
#define MAX_COLS L5_MAX_COLS

// L5雷达帧
typedef struct
{
    int ToF_timestamps;                    // 时间戳
    int ToF_distances[MAX_ROWS][MAX_COLS]; // 距离矩阵，单位mm
} L5_LidarDataFrame;

// IMU数据帧
typedef struct
{
    int IMU_timestamps; // 时间戳
    double roll;        // 角度制
    double pitch;
    double yaw;
    double x;           // 单位：m
    double y;
    double z;
} IMUDataFrame;

// 位姿结构体
typedef struct
{
    double x, y, z, roll, pitch, yaw;   // 单位：mm，角度制
} Pos;


// 定义 Point 结构体，用于表示3D坐标
typedef struct
{
    double x; // x 坐标，单位：mm
    double y; // y 坐标
    double z; // z 坐标
} Point;

// 点云帧
typedef struct
{
    int ToF_timestamps;                     // 时间戳
    Point ToF_position[MAX_ROWS][MAX_COLS]; // 3D点矩阵，每个元素是一个 Point
} PointCloud;


// 将 lidar 数据帧的 ToF_distances 转换为 PointCloudData 的 3D 坐标
void convertToPointCloud(int distances[MAX_ROWS][MAX_COLS], Point pointCloud[MAX_ROWS][MAX_COLS]);

void printPointCloud(PointCloud pointcloud);

#endif