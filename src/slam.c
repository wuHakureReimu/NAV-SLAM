#include <stdio.h>
#include <stddef.h>
#include "slam.h"
#include "math.h"
#include "kdtree.h"
#include "pointcloud.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

// 计算曲率以及特征（仅边缘点）提取
void extract_feature(PointCloud *lidarPointCloud, int feature[MAX_ROWS][MAX_COLS]) {
    const int smooth_window = 2;  // 平滑窗口大小，可调整
    
    // 遍历每个点（跳过边缘的点）
    for (int i = 0; i < MAX_ROWS; i++) {
        for (int j = smooth_window; j < MAX_COLS - smooth_window; j++) {
            // 获取当前点
            Point current_point = lidarPointCloud->ToF_position[i][j];
            
            // 计算邻域点的距离加权曲率
            double sum_dist = 0.0;
            int count = 0;
            // 计算前向和后向邻域点的距离
            for (int k = -smooth_window; k <= smooth_window; k++) {
                if (k == 0) continue;
                
                Point neighbor_point = lidarPointCloud->ToF_position[i][j + k];
                double dx = current_point.x - neighbor_point.x;
                double dy = current_point.y - neighbor_point.y;
                double dz = current_point.z - neighbor_point.z;
                
                double dist_sq = dx*dx + dy*dy + dz*dz;
                sum_dist += sqrt(dist_sq);
                count++;
            }
            // 计算平均距离
            double avg_dist = (count > 0) ? sum_dist / count : 0;
            
            // 计算曲率
            double curvature = 0.0;
            // 基于距离方差计算
            if (count > 0 && avg_dist > 0) {
                double sum_var = 0.0;
                for (int k = -smooth_window; k <= smooth_window; k++) {
                    if (k == 0) continue;
                    Point neighbor_point = lidarPointCloud->ToF_position[i][j + k];
                    double dx = current_point.x - neighbor_point.x;
                    double dy = current_point.y - neighbor_point.y;
                    double dz = current_point.z - neighbor_point.z;
                    double dist = sqrt(dx*dx + dy*dy + dz*dz);
                    
                    sum_var += (dist - avg_dist) * (dist - avg_dist);
                }
                curvature = sum_var / count / (avg_dist * avg_dist + 1e-6f);
            }
            
            // 提取特征（仅边缘点）
            if (curvature > 0.1) feature[i][j] = 1;
        }
    }
}

// 辅助函数：扁平化PointCloud并筛选出特征点
void flattenPoints(Point rowPoints[MAX_COLS], int rowFeature[MAX_COLS], Point flattenedPoints[MAX_COLS], size_t *numPoints)
{
    *numPoints = 0;
    for (int i = 0; i < MAX_COLS; ++i) {
        if (rowFeature[i] == 1) {
            flattenedPoints[*numPoints] = rowPoints[i];
            (*numPoints)++;
        }
    }

#ifdef DEBUG_PRINT
    printf("扁平化后的特征点：\n");
    for (size_t i = 0; i < *numPoints; ++i)
    {
        printf("Point[%zu]: x = %.3f, y = %.3f, z = %.3f\n", i, flattenedPoints[i].x, flattenedPoints[i].y, flattenedPoints[i].z);
    }
#endif
}

// 辅助函数：计算变换，单位mm
void compute_posdiff(Pos *pos_now, Pos *pos_last, double pos_diff[6])
{
    pos_diff[0] = pos_now->x - pos_last->x;
    pos_diff[1] = pos_now->y - pos_last->y;
    pos_diff[2] = pos_now->z - pos_last->z;
    pos_diff[3] = pos_now->roll - pos_last->roll;
    pos_diff[4] = pos_now->pitch - pos_last->pitch;
    pos_diff[5] = pos_now->yaw - pos_last->yaw;
}

// 辅助函数：计算旋转矩阵（输入弧度制）
void getRotationMatrix(double roll, double pitch, double yaw, double R[3][3])
{
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

// 辅助函数：根据位移 transform 映射到上一帧
void mapCoordinatesToLastFrame(PointCloud *globalPointCloudData, double transform[3], PointCloud *positionInLastFrame)
{
    // 遍历每一行和每一列
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            // 将当前坐标减去transform（即计算位置在上一帧的坐标）
            positionInLastFrame->ToF_position[row][col].x = globalPointCloudData->ToF_position[row][col].x - transform[0];
            positionInLastFrame->ToF_position[row][col].y = globalPointCloudData->ToF_position[row][col].y - transform[1];
            positionInLastFrame->ToF_position[row][col].z = globalPointCloudData->ToF_position[row][col].z - transform[2];
        }
    }
}

// 用第一帧数据初始化
void init_slam(SLAM_attr *attr, Pos pos, PointCloud *lidarPointCloud)
{
    // 初始化frameCount
    attr->frameCount = 0;

    double R[3][3];
    getRotationMatrix(DEG2RAD(pos.roll), DEG2RAD(pos.pitch), DEG2RAD(pos.yaw), R);

    // 初始化全局点云第0帧
    attr->globalPointCloud[attr->frameCount].ToF_timestamps = lidarPointCloud->ToF_timestamps;
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            double lidar_x = lidarPointCloud->ToF_position[row][col].x;
            double lidar_y = lidarPointCloud->ToF_position[row][col].y;
            double lidar_z = lidarPointCloud->ToF_position[row][col].z;

            double rotated_x = R[0][0] * lidar_x + R[0][1] * lidar_y + R[0][2] * lidar_z;
            double rotated_y = R[1][0] * lidar_x + R[1][1] * lidar_y + R[1][2] * lidar_z;
            double rotated_z = R[2][0] * lidar_x + R[2][1] * lidar_y + R[2][2] * lidar_z;

            // 单位：mm
            attr->globalPointCloud[attr->frameCount].ToF_position[row][col].x = pos.x + rotated_x;
            attr->globalPointCloud[attr->frameCount].ToF_position[row][col].y = pos.y + rotated_y;
            attr->globalPointCloud[attr->frameCount].ToF_position[row][col].z = pos.z + rotated_z;
        }
    }

    // 特征提取
    int feature[MAX_ROWS][MAX_COLS] = {0}; // 特征矩阵
    extract_feature(lidarPointCloud, feature);

    // 初始化第0帧kdtree
    for (int row = 0; row < MAX_ROWS; ++row) {
        Point flattenedPoints[MAX_COLS];
        size_t numPoints;
        flattenPoints(attr->globalPointCloud[attr->frameCount].ToF_position[row], feature[row], flattenedPoints, &numPoints);
        attr->kdtree_lastframe[row] = buildKDTree(flattenedPoints, numPoints, 0); // 对每一行数据单独构建KD-Tree
    }
    
    attr->frameCount++;
}

// 配准定位
Pos slam_localization(SLAM_attr *attr, PointCloud *lidarPointCloud, Pos pos_predict, Pos pos_last)
{
    double R[3][3];
    getRotationMatrix(DEG2RAD(pos_predict.roll), DEG2RAD(pos_predict.pitch), DEG2RAD(pos_predict.yaw), R);
    
    // 1. 提取当前帧特征    
    int feature[MAX_ROWS][MAX_COLS] = {0};
    extract_feature(lidarPointCloud, feature);
    
    // 2. 把当前帧点云映射到上一帧，得到positionInLastframe（全局坐标系），便于寻找最近点
    double transform[6];
    compute_posdiff(&pos_predict, &pos_last, transform);
    
    // 转换当前帧点云到全局坐标系
    PointCloud transformed_pointcloud;
    for (int row = 0; row < MAX_ROWS; row++) {
        for (int col = 0; col < MAX_COLS; col++) {
            double lidar_x = lidarPointCloud->ToF_position[row][col].x;
            double lidar_y = lidarPointCloud->ToF_position[row][col].y;
            double lidar_z = lidarPointCloud->ToF_position[row][col].z;

            double rotated_x = R[0][0] * lidar_x + R[0][1] * lidar_y + R[0][2] * lidar_z;
            double rotated_y = R[1][0] * lidar_x + R[1][1] * lidar_y + R[1][2] * lidar_z;
            double rotated_z = R[2][0] * lidar_x + R[2][1] * lidar_y + R[2][2] * lidar_z;

            transformed_pointcloud.ToF_position[row][col].x = pos_predict.x + rotated_x;
            transformed_pointcloud.ToF_position[row][col].y = pos_predict.y + rotated_y;
            transformed_pointcloud.ToF_position[row][col].z = pos_predict.z + rotated_z;
        }
    }
    
    PointCloud positionInLastFrame;
    mapCoordinatesToLastFrame(&transformed_pointcloud, transform, &positionInLastFrame);

    // 3. 配准
    // 外部迭代循环，设置20次迭代
    NeighborResult result[100];
    int CPcount = 0;

    // 计算每个维度（x, y, z）的梯度
    double learningRate = 0.1; // 学习啦
    double tolerance = 1e-6; // 收敛容忍度
    double previousTotalError = 0;
    double totalError = 0;
    int validGradientCount = 0;
    // ADAM
    double m[3] = {0.0, 0.0, 0.0};
    double v[3] = {0.0, 0.0, 0.0};
    double beta1 = 0.9;
    double beta2 = 0.999;
    double epsilon = 1e-8;
    
    for (int iter = 0; iter < 200; ++iter)
    {
        // 3.1 找最近点
        if (iter % 200 == 0)  // 注意：这里应该是iter == 0，或者每N次迭代重新搜索最近点
        {
            int flag = 0;
            for (int row = 0; row < MAX_ROWS; ++row) {
                for (int col = 0; col < MAX_COLS; ++col) {
                    if (feature[row][col] == 1)
                    { // 只处理特征点
                        // 最近邻查找
                        Point currentPoint = positionInLastFrame.ToF_position[row][col];
                        Point CurnearestPoint;
                        double bestDist = INFINITY;              // 初始设为一个非常大的值
                        nearestNeighborSearch(attr->kdtree_lastframe[row], &currentPoint, &CurnearestPoint, &bestDist, 0);

                        // 放入最近邻数组
                        if (flag == CPcount)
                        {
                            result[CPcount].oriPoint = transformed_pointcloud.ToF_position[row][col];
                            result[CPcount].nearestPoint = CurnearestPoint;
                            result[CPcount].distance = bestDist;
                            CPcount++;
                            continue;
                        }

                        // 每一行的其他点
                        int found = 0;
                        for (int i = flag; i < CPcount; i++)
                        {
                            if ((result[i].nearestPoint.x == CurnearestPoint.x) &&
                                (result[i].nearestPoint.y == CurnearestPoint.y) &&
                                (result[i].nearestPoint.z == CurnearestPoint.z))
                            {
                                if (result[i].distance > bestDist)
                                {
                                    result[i].oriPoint = transformed_pointcloud.ToF_position[row][col];
                                    result[i].nearestPoint = CurnearestPoint;
                                    result[i].distance = bestDist;
                                }
                                found = 1;
                                break;
                            }
                        }
                        
                        if (!found) {
                            result[CPcount].oriPoint = transformed_pointcloud.ToF_position[row][col];
                            result[CPcount].nearestPoint = CurnearestPoint;
                            result[CPcount].distance = bestDist;
                            CPcount++;
                        }
                    }
                }
                flag = CPcount;
            }

#ifdef DEBUG_PRINT
            printf("时间戳: %d\n", imuData.IMU_timestamps);
            printf("时间戳: %zu\n", attr->frameCount);
            for (int i = 0; i < CPcount; i++)
            {
                printf("Result %d:\n", i);
                printf("  Original Point: (%.3f, %.3f, %.3f)\n", result[i].oriPoint.x, result[i].oriPoint.y, result[i].oriPoint.z);
                printf("  project Point: (%.3f, %.3f, %.3f)\n", result[i].oriPoint.x - transform[0], result[i].oriPoint.y - transform[1], result[i].oriPoint.z - transform[2]);
                printf("  Nearest Point: (%.3f, %.3f, %.3f)\n", result[i].nearestPoint.x, result[i].nearestPoint.y, result[i].nearestPoint.z);
                printf("ErrDistance = %.3f m\n", result[i].distance);
            }
#endif
        }

        // 3.2 计算误差 d
        double ErrDistance[100];
        for (int i = 0; i < CPcount; i++)
        {
            // 注意：这里计算的是映射后的点与最近点之间的距离
            ErrDistance[i] = sqrt(pow((result[i].oriPoint.x - transform[0]) - result[i].nearestPoint.x, 2) +
                                  pow((result[i].oriPoint.y - transform[1]) - result[i].nearestPoint.y, 2) +
                                  pow((result[i].oriPoint.z - transform[2]) - result[i].nearestPoint.z, 2));
        }

#ifdef DEBUG_PRINT
        printf("时间戳: %d\n", imuData.IMU_timestamps);
        for (int i = 0; i < CPcount; i++)
        {
            printf("ErrDistance[%d] = %.3f m\n", i, ErrDistance[i]);
        }
#endif

        // 更稳健的梯度下降实现
        double gradient[3] = {0.0, 0.0, 0.0};
        totalError = 0;
        validGradientCount = 0;

        // 使用平方误差，避免开方运算
        for (int i = 0; i < CPcount; i++)
        {
            double dx = (result[i].oriPoint.x - transform[0]) - result[i].nearestPoint.x;
            double dy = (result[i].oriPoint.y - transform[1]) - result[i].nearestPoint.y;
            double dz = (result[i].oriPoint.z - transform[2]) - result[i].nearestPoint.z;

            double dist_sq = dx * dx + dy * dy + dz * dz;
            totalError += dist_sq;

            // 梯度计算：E = 1/2 * sum(dist^2)，dE/dtransform = -sum(dx, dy, dz)
            gradient[0] -= dx; // 负梯度方向
            gradient[1] -= dy;
            gradient[2] -= dz;
            validGradientCount++;
        }

        // 判断收敛
        if (fabs(totalError - previousTotalError) < tolerance)
        {
            printf("收敛，停止迭代！\n");
            break;
        }
        previousTotalError = totalError;

        if (validGradientCount > 0)
        {
            gradient[0] /= validGradientCount;
            gradient[1] /= validGradientCount;
            gradient[2] /= validGradientCount;
        }

        // ADAM优化器更新
        int t = iter + 1;
        for (int j = 0; j < 3; j++)
        {
            // 更新一阶矩估计
            m[j] = beta1 * m[j] + (1 - beta1) * gradient[j];
            // 更新二阶矩估计
            v[j] = beta2 * v[j] + (1 - beta2) * gradient[j] * gradient[j];

            // 计算偏差修正后的矩估计
            double m_hat = m[j] / (1 - pow(beta1, t));
            double v_hat = v[j] / (1 - pow(beta2, t));

            // 使用Adam更新规则更新transform
            transform[j] -= learningRate * m_hat / (sqrt(v_hat) + epsilon);
        }

        printf("Iteration %d, Total Error: %.6f\n", iter, totalError);
    }
    
    Pos pos_modified;
    pos_modified.x = pos_last.x + transform[0];
    pos_modified.y = pos_last.y + transform[1];
    pos_modified.z = pos_last.z + transform[2];
    pos_modified.roll = pos_last.roll + transform[3];   // 上面的配准算法尚未实现角度变换的配准计算
    pos_modified.pitch = pos_last.pitch + transform[4];
    pos_modified.yaw = pos_last.yaw + transform[5];

    return pos_modified;
}

// 建图
void slam_mapping(SLAM_attr *attr, Pos pos, PointCloud *lidarPointCloud)
{
    attr->globalPointCloud[attr->frameCount].ToF_timestamps = lidarPointCloud->ToF_timestamps;

    // 根据精确位姿计算旋转变换
    double R[3][3];
    getRotationMatrix(DEG2RAD(pos.roll), DEG2RAD(pos.pitch), DEG2RAD(pos.yaw), R);

    // 将雷达坐标系下的点云转换到全局坐标系下
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            double lidar_x = lidarPointCloud->ToF_position[row][col].x;
            double lidar_y = lidarPointCloud->ToF_position[row][col].y;
            double lidar_z = lidarPointCloud->ToF_position[row][col].z;

            double rotated_x = R[0][0] * lidar_x + R[0][1] * lidar_y + R[0][2] * lidar_z;
            double rotated_y = R[1][0] * lidar_x + R[1][1] * lidar_y + R[1][2] * lidar_z;
            double rotated_z = R[2][0] * lidar_x + R[2][1] * lidar_y + R[2][2] * lidar_z;

            attr->globalPointCloud[attr->frameCount].ToF_position[row][col].x = pos.x + rotated_x;
            attr->globalPointCloud[attr->frameCount].ToF_position[row][col].y = pos.y + rotated_y;
            attr->globalPointCloud[attr->frameCount].ToF_position[row][col].z = pos.z + rotated_z;
        }
    }

    // 构建kdtree，供下一帧配准使用    
    int feature[MAX_ROWS][MAX_COLS] = {0};
    extract_feature(lidarPointCloud, feature);
    
    for (int row = 0; row < MAX_ROWS; ++row) {
        Point flattenedPoints[MAX_COLS];
        size_t numPoints;
        flattenPoints(attr->globalPointCloud[attr->frameCount].ToF_position[row], feature[row], flattenedPoints, &numPoints);
        attr->kdtree_lastframe[row] = buildKDTree(flattenedPoints, numPoints, 0); // 对每一行数据单独构建KD-Tree
    }

    // 每次建完图，frameCount +1
    attr->frameCount++;
}