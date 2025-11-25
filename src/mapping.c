#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "kdtree.h"
#include "pointcloud.h"

// 3个自由度的位姿变化：xyz 3个方向的位移变化
KDNode *lastKdTreeRoot[MAX_ROWS] = {NULL};   // 用来存储上一帧每一行的 KD-Tree 根节点
PointCloud *lastGlobalPointCloudData = NULL; // 上一帧的全局点云数据
IMUDataFrame lastModifiedPosition;           // 上一帧的修正后IMU位置

// 是否是第一帧
bool isFirstFrame = true;

// 全局变量存储当前帧的位姿信息
PoseInfo currentPose;

// 计算曲率
int computeCurvature(int i, int row[MAX_COLS])
{
    int left = (i > 0) ? abs(row[i] - row[i - 1]) : 0;
    int right = (i < MAX_COLS - 1) ? abs(row[i] - row[i + 1]) : 0;
    if ((i <= 0) || (i >= MAX_COLS - 1))
        return 0;
    return left + right;
}

// 扁平化PointCloud并筛选出特征点
void flattenPoints(Point rowPoints[MAX_COLS], int FeatureMatrix[MAX_COLS], Point *flattenedPoints, size_t *numPoints)
{
    *numPoints = 0;

    for (int i = 0; i < MAX_COLS; ++i)
    {
        if (FeatureMatrix[i] == 1)
        {
            flattenedPoints[*numPoints] = rowPoints[i];
            (*numPoints)++;
        }
    }

#ifdef DEBUG_PRINT
    printf("扁平化后的特征点：\n");
    for (size_t i = 0; i < *numPoints; ++i)
    {
        printf("Point[%zu]: x = %.2f, y = %.2f, z = %.2f\n", i, flattenedPoints[i].x, flattenedPoints[i].y, flattenedPoints[i].z);
    }
#endif
}

// 计算位移的函数
void computeDisplacement(IMUDataFrame *imuData, IMUDataFrame *lastModifiedPosition, double transform[3])
{
    // 计算位移: imuData的坐标与上一帧的位置进行差值
    transform[0] = imuData->x * 1000 - lastModifiedPosition->x; // 计算x轴位移
    transform[1] = imuData->y * 1000 - lastModifiedPosition->y; // 计算y轴位移
    transform[2] = imuData->z * 1000 - lastModifiedPosition->z; // 计算z轴位移

// 打印位移以进行验证
#ifdef DEBUG_PRINT
    printf("x: %.6f, y: %.6f, z: %.6f\n", transform[0], transform[1], transform[2]);
#endif
}

// 根据位移 transform 进行坐标映射
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

void laserCloudHandler(LidarDataFrame lidarDataFrame, IMUDataFrame imuData, PointCloud *globalPointCloudData, size_t *frameCount)
{
    IMUDataFrame CurModifiedPosition; // pm(i) 修正后的imu全局位置
    PointCloud PointCloudData_TOF;    // 相对于TOF的3D坐标
    double transform[3] = {0};        // 3个自由度的变换

    PointCloudData_TOF.ToF_timestamps = lidarDataFrame.ToF_timestamps;
    globalPointCloudData->ToF_timestamps = lidarDataFrame.ToF_timestamps;

    // 设置当前位姿信息中的IMU原始坐标（转换为毫米）
    currentPose.imu_x = imuData.x * 1000;
    currentPose.imu_y = imuData.y * 1000;
    currentPose.imu_z = imuData.z * 1000;

    // 深度图转换为相对于tof传感器的相对坐标
    convertToPointCloud(lidarDataFrame.ToF_distances, PointCloudData_TOF.ToF_position);

#ifdef DEBUG_PRINT
    printf("时间戳: %zu\n", *frameCount);
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            printf("Point[%d][%d]: x = %.2f, y = %.2f, z = %.2f,distance = %d\n",
                   row, col,
                   PointCloudData_TOF.ToF_position[row][col].x,
                   PointCloudData_TOF.ToF_position[row][col].y,
                   PointCloudData_TOF.ToF_position[row][col].z,
                   lidarDataFrame.ToF_distances[row][col]);
        }
    }
#endif

    // 根据imu提供的全局无人机坐标计算数据点的3维绝对坐标
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            globalPointCloudData->ToF_position[row][col].x = (imuData.x * 1000) + PointCloudData_TOF.ToF_position[row][col].x;
            globalPointCloudData->ToF_position[row][col].y = (imuData.y * 1000) + PointCloudData_TOF.ToF_position[row][col].y;
            globalPointCloudData->ToF_position[row][col].z = (imuData.z * 1000) + PointCloudData_TOF.ToF_position[row][col].z;
        }
    }

    // 特征提取
    int curvatureMatrix[MAX_ROWS][MAX_COLS];     // 曲率矩阵
    int FeatureMatrix[MAX_ROWS][MAX_COLS] = {0}; // 初始化特征矩阵

    // 计算曲率矩阵
    for (int i = 0; i < MAX_ROWS; ++i)
    {
        for (int j = 0; j < MAX_COLS; ++j)
        {
            curvatureMatrix[i][j] = computeCurvature(j, lidarDataFrame.ToF_distances[i]);
        }
    }

#ifdef DEBUG_PRINT
    printf("时间戳: %zu\n", *frameCount);
    for (int i = 0; i < MAX_ROWS; ++i)
    {
        for (int j = 0; j < MAX_COLS; ++j)
        {
            printf("%d ", curvatureMatrix[i][j]);
        }
        printf("\n");
    }
#endif

    // 特征提取
    for (int i = 0; i < MAX_ROWS; ++i)
    {
        for (int j = 0; j < MAX_COLS; ++j)
        {
            if (curvatureMatrix[i][j] > 500 && lidarDataFrame.ToF_distances[i][j] < 1000)
            { // 前景的边缘
                FeatureMatrix[i][j] = 1;
            }
        }
    }

#ifdef DEBUG_PRINT
    printf("时间戳: %zu\n", *frameCount);
    for (int i = 0; i < MAX_ROWS; ++i)
    {
        for (int j = 0; j < MAX_COLS; ++j)
        {
            printf("%d", FeatureMatrix[i][j]);
        }
        printf("\n");
    }
#endif

    // loam 的场景重建 mapping

    // 扁平化每一行的特征点并构建KD-Tree
    KDNode *kdTreeRoot[MAX_ROWS];

    printf("时间戳: %zu\n", *frameCount);
    // 第一帧
    if (isFirstFrame)
    {
        isFirstFrame = false;

        // 构建kd-tree，仅仅包含特征点
        for (int row = 0; row < MAX_ROWS; ++row)
        {
            Point flattenedPoints[MAX_COLS];
            size_t numPoints = 0;
            flattenPoints(globalPointCloudData->ToF_position[row], FeatureMatrix[row], flattenedPoints, &numPoints);
            kdTreeRoot[row] = buildKDTree(flattenedPoints, numPoints, 0); // 对每一行数据单独构建KD-Tree
        }

        // 保存上一帧
        for (int row = 0; row < MAX_ROWS; ++row)
        {
            // 保存上一帧每一行的KD-Tree根节点
            lastKdTreeRoot[row] = kdTreeRoot[row];
        }
        lastGlobalPointCloudData = globalPointCloudData; // 将当前的全局点云数据赋值给lastGlobalPointCloudData

        lastModifiedPosition.IMU_timestamps = imuData.IMU_timestamps; // 将当前的IMU数据赋值给lastIMUdata
        lastModifiedPosition.x = imuData.x * 1000;
        lastModifiedPosition.y = imuData.y * 1000;
        lastModifiedPosition.z = imuData.z * 1000;

        // 第一帧时，修正后的坐标等于IMU原始坐标
        currentPose.modified_x = currentPose.imu_x;
        currentPose.modified_y = currentPose.imu_y;
        currentPose.modified_z = currentPose.imu_z;

        return; // Pimu1 = Pm1 imu提供的坐标直接作为全局坐标，不修正
    }

    // 第2帧及以后
    // 1. Tr(i) = Pimu(i) - Pm(i-1)  初始位移 imu提供的i帧坐标 - 修正后的 i-1 帧坐标
    computeDisplacement(&imuData, &lastModifiedPosition, transform); // 计算位移，transform单位：mm

    // 2. Pt(i) = Tr(i) + Pimu(i)  第 i 帧的点按照这个位移反映射回 i - 1帧的位置 Pt(i)
    PointCloud positionInLastFrame;
    // 根据 transform 对坐标进行映射，映射回上一帧
    mapCoordinatesToLastFrame(globalPointCloudData, transform, &positionInLastFrame); // positionInLastFrame单位：mm

#ifdef DEBUG_PRINT
                                                                                      // 打印 globalPointCloudData
    printf("时间戳: %d\n", imuData.IMU_timestamps);
    printf("当前帧的 globalPointCloudData:\n");
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            printf("globalPointCloudData[%d][%d]: x = %.2f, y = %.2f, z = %.2f\n",
                   row, col,
                   globalPointCloudData->ToF_position[row][col].x,
                   globalPointCloudData->ToF_position[row][col].y,
                   globalPointCloudData->ToF_position[row][col].z);
        }
    }

    // 打印 positionInLastFrame
    printf("映射后的 positionInLastFrame:\n");
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            if (FeatureMatrix[row][col] == 1)
            {
                printf("positionInLastFrame[%d][%d]: x = %.6f, y = %.6f, z = %.6f\n",
                       row, col,
                       positionInLastFrame.ToF_position[row][col].x,
                       positionInLastFrame.ToF_position[row][col].y,
                       positionInLastFrame.ToF_position[row][col].z);
            }
        }
    }
#endif

    // 外部迭代循环，设置20次迭代
    NeighborResult result[100];
    int CPcount = 0;

    // 计算每个维度（x, y, z）的梯度
    double learningRate = 2; // 设置学习率
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
        // 3. 构造几何关系，误差函数
        // 3.1 使用 kd-tree（lastKdTreeRoot），找到 映射后的i帧中所有特征点的 与其在同一行的 1 个i-1 帧中的最近邻点
        if (iter % 200 == 0)
        {
            int flag = 0;
            for (int row = 0; row < MAX_ROWS; ++row)
            {

                for (int col = 0; col < MAX_COLS; ++col)
                {
                    if (FeatureMatrix[row][col] == 1)
                    { // 只处理特征点

                        // 最近邻查找
                        Point currentPoint = positionInLastFrame.ToF_position[row][col];
                        Point CurnearestPoint = currentPoint;
                        double bestDist = INFINITY;                                                                // 初始设为一个非常大的值
                        nearestNeighborSearch(lastKdTreeRoot[row], &currentPoint, &CurnearestPoint, &bestDist, 0); // 修改为针对每一行的KDTree

                        // 放入最近邻数组
                        // 每一行的第一个点
                        if (flag == CPcount)
                        {
                            result[CPcount].oriPoint = globalPointCloudData->ToF_position[row][col];
                            result[CPcount].nearestPoint = CurnearestPoint;
                            result[CPcount].distance = euclideanDistance(currentPoint, CurnearestPoint);
                            CPcount++;
                            continue;
                        }

                        // 每一行的其他点
                        for (int i = flag; i < CPcount; i++)
                        {
                            if ((result[i].nearestPoint.x == CurnearestPoint.x) &&
                                (result[i].nearestPoint.y == CurnearestPoint.y) &&
                                (result[i].nearestPoint.z == CurnearestPoint.z))
                            {
                                if (result[i].distance > euclideanDistance(currentPoint, CurnearestPoint))
                                {
                                    result[i].oriPoint = globalPointCloudData->ToF_position[row][col];
                                    result[i].nearestPoint = CurnearestPoint;
                                    result[i].distance = euclideanDistance(currentPoint, CurnearestPoint);
                                    break;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                result[CPcount].oriPoint = globalPointCloudData->ToF_position[row][col];
                                result[CPcount].nearestPoint = CurnearestPoint;
                                result[CPcount].distance = euclideanDistance(currentPoint, CurnearestPoint);
                                CPcount++;
                            }
                        }

                    } // result单位：mm
                }
                flag = CPcount;
            } // end for

#ifdef DEBUG_PRINT
            printf("时间戳: %d\n", imuData.IMU_timestamps);
            printf("时间戳: %zu\n", *frameCount);
            for (int i = 0; i < CPcount; i++)
            {
                printf("Result %d:\n", i);
                printf("  Original Point: (%.2f, %.2f, %.2f)\n", result[i].oriPoint.x, result[i].oriPoint.y, result[i].oriPoint.z);
                printf("  project Point: (%.2f, %.2f, %.2f)\n", result[i].oriPoint.x - transform[0], result[i].oriPoint.y - transform[1], result[i].oriPoint.z - transform[2]);
                printf("  Nearest Point: (%.2f, %.2f, %.2f)\n", result[i].nearestPoint.x, result[i].nearestPoint.y, result[i].nearestPoint.z);
                printf("ErrDistance = %.2f mm\n", result[i].distance);
            }
#endif
        }

        // 3.2 计算误差 d （具体的定义方式有待商榷）
        double ErrDistance[100];
        for (int i = 0; i < CPcount; i++)
        {
            ErrDistance[i] = sqrt(pow((result[i].oriPoint.x - transform[0]) - result[i].nearestPoint.x, 2) +
                                  pow((result[i].oriPoint.y - transform[1]) - result[i].nearestPoint.y, 2) +
                                  pow((result[i].oriPoint.z - transform[2]) - result[i].nearestPoint.z, 2));
        }

#ifdef DEBUG_PRINT
        printf("时间戳: %d\n", imuData.IMU_timestamps);
        for (int i = 0; i < CPcount; i++)
        {
            printf("ErrDistance[%d] = %.2f mm\n", i, ErrDistance[i]);
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
        // 自适应学习率 - 根据梯度大小调整
        /*double grad_norm = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] + gradient[2] * gradient[2]);
        if (grad_norm > 1e-6)
        {
            double adaptive_lr = learningRate / grad_norm; // 归一化
            transform[0] += adaptive_lr * gradient[0];
            transform[1] += adaptive_lr * gradient[1];
            transform[2] += adaptive_lr * gradient[2];
        }*/
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
    // 计算pm(i) = pm(i-1) + Tr(i) 修正后的 i 帧无人机位置 = 修正后的 i-1 帧无人机位置 + 迭代优化后的位移
    CurModifiedPosition.IMU_timestamps = imuData.IMU_timestamps;
    CurModifiedPosition.x = lastModifiedPosition.x + transform[0];
    CurModifiedPosition.y = lastModifiedPosition.y + transform[1];
    CurModifiedPosition.z = lastModifiedPosition.z + transform[2];

    // 设置当前位姿信息中的修正后坐标
    currentPose.modified_x = CurModifiedPosition.x;
    currentPose.modified_y = CurModifiedPosition.y;
    currentPose.modified_z = CurModifiedPosition.z;

    // 计算修正后的点云坐标
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            globalPointCloudData->ToF_position[row][col].x = (CurModifiedPosition.x) + PointCloudData_TOF.ToF_position[row][col].x;
            globalPointCloudData->ToF_position[row][col].y = (CurModifiedPosition.y) + PointCloudData_TOF.ToF_position[row][col].y;
            globalPointCloudData->ToF_position[row][col].z = (CurModifiedPosition.z) + PointCloudData_TOF.ToF_position[row][col].z;
        }
    }

    // 构建kd-tree供下一帧使用，仅仅包含特征点
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        Point flattenedPoints[MAX_COLS];
        size_t numPoints = 0;
        flattenPoints(globalPointCloudData->ToF_position[row], FeatureMatrix[row], flattenedPoints, &numPoints);
        kdTreeRoot[row] = buildKDTree(flattenedPoints, numPoints, 0); // 对每一行数据单独构建KD-Tree
    }

    // 保存上一帧
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        // 保存上一帧每一行的KD-Tree根节点
        lastKdTreeRoot[row] = kdTreeRoot[row];
    }

    lastGlobalPointCloudData = globalPointCloudData; // 将当前的全局点云数据赋值给lastGlobalPointCloudData
    lastModifiedPosition = CurModifiedPosition;
}
