

#include <math.h>
#include "pointcloud.h"


// 将 lidar 数据帧的 ToF_distances 转换为 PointCloudData 的 3D 坐标
void convertToPointCloud(int distances[MAX_ROWS][MAX_COLS], Point pointCloud[MAX_ROWS][MAX_COLS]) {
    // 定义水平和垂直视场角（度）
    const double fov_h = 45.0;  // 水平视场角
    const double fov_v = 45.0;  // 垂直视场角

    // 计算每个方向上的角度步长（度）
    const double theta_step_deg = fov_h / (MAX_COLS - 1);  // 水平角度步长
    const double phi_step_deg = fov_v / (MAX_ROWS - 1);    // 垂直角度步长

    // 遍历每一行（扫描线）
    for (int j = 0; j < MAX_ROWS; ++j) {
        // 遍历每一列
        for (int i = 0; i < MAX_COLS; ++i) {
            double distance = distances[j][i];

            // 假设距离值为0时表示无效数据
            if (distance <= 0) {
                pointCloud[j][i].x = pointCloud[j][i].y = pointCloud[j][i].z = 0.0;
                continue;  // 跳过无效数据
            }

            // 计算水平和垂直角度（弧度）
            double theta = -fov_h / 2.0 + i * theta_step_deg;  // 水平角度（度）
            double phi = -fov_v / 2.0 + j * phi_step_deg;     // 垂直角度（度）

            // 转换为弧度
            theta = theta * M_PI / 180.0;  // 转换为弧度
            phi = phi * M_PI / 180.0;      // 转换为弧度

            // 计算3D坐标
            double x = distance ;
            double y = -(distance ) * tan(theta);  // 水平坐标
            double z = -(distance ) * tan(phi);    // 垂直坐标

            // 存储转换后的坐标
            pointCloud[j][i].x = x;
            pointCloud[j][i].y = y;
            pointCloud[j][i].z = z;
        }
    }
}

void printPointCloud(PointCloud pointcloud)
{
    for (int i = 0; i < MAX_ROWS; i++) {
        for (int j = 0; j < MAX_COLS; j++) {
            Point point = pointcloud.ToF_position[i][j];
            printf("point %d: (%f, %f, %f) \n", i * MAX_ROWS + j, point.x, point.y, point.z);
        }
    }
}
