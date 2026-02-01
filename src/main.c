
#include <stdio.h>
#include <jansson.h>

#include "ekf.h"
#include "slam.h"
#include "pointcloud.h"

#define DEBUG_PRINT
#define FILE_PRINT

// 读取Lidar数据函数
void LidarProcessData(const char *filename, LidarDataFrame *lidarData, size_t *lidarCount)
{
    FILE *file = fopen(filename, "r");
    if (file == NULL)
    {
        perror("无法打开文件");
        return;
    }

    json_error_t error;
    json_t *root = json_loadf(file, 0, &error);
    fclose(file);
    if (!root)
    {
        printf("JSON解析错误: %s\n", error.text);
        return;
    }

    // 检查root是否是一个数组，且数组中有元素
    if (json_is_array(root) && json_array_size(root) > 0)
    {
        size_t array_size = json_array_size(root);

        // 遍历整个数组中的每个数据对象
        for (size_t i = 0; i < array_size; i++)
        {
            json_t *obj = json_array_get(root, i); // 获取数组中的第 i 个对象
            if (json_is_object(obj))
            {

                // 读取 "time_main" 字段
                json_t *time_main = json_object_get(obj, "time_main");
                if (json_is_integer(time_main))
                {
                    lidarData[*lidarCount].ToF_timestamps = json_integer_value(time_main);
                }

                // 解析 "distance" 数组并存入矩阵
                json_t *distance = json_object_get(obj, "distance");
                if (json_is_array(distance))
                {
                    size_t index = 0;
                    json_t *value;
                    json_array_foreach(distance, index, value)
                    {
                        if (json_is_integer(value) && index < MAX_ROWS * MAX_COLS)
                        {
                            int row = index / MAX_COLS;
                            int col = index % MAX_COLS;
                            lidarData[*lidarCount].ToF_distances[row][col] = json_integer_value(value);
                        }
                    }
                }
            }

            // 增加 lidarCount 计数器，准备存储下一个数据对象
            (*lidarCount)++;
        }
    }

    json_decref(root);
}

// 读取IMU数据函数
void IMUProcessData(const char *filename, IMUDataFrame *imuData, size_t *imuCount)
{
    FILE *file = fopen(filename, "r");
    if (file == NULL)
    {
        perror("无法打开文件");
        return;
    }

    json_error_t error;
    json_t *root = json_loadf(file, 0, &error);
    fclose(file);
    if (!root)
    {
        printf("JSON解析错误: %s\n", error.text);
        return;
    }

    if (json_is_array(root) && json_array_size(root) > 0)
    {
        size_t array_size = json_array_size(root);

        // 遍历整个数组中的每个数据对象
        for (size_t i = 0; i < array_size; i++)
        {
            json_t *obj = json_array_get(root, i); // 获取数组中的第 i 个对象
            if (json_is_object(obj))
            {

                // 解析 "time_main" 字段
                json_t *time_main = json_object_get(obj, "time_main");
                if (json_is_integer(time_main))
                {
                    imuData[*imuCount].IMU_timestamps = json_integer_value(time_main);
                }

                // 解析 "params" 数组
                json_t *params = json_object_get(obj, "params");
                if (json_is_array(params) && json_array_size(params) == 6)
                {
                    imuData[*imuCount].roll = json_real_value(json_array_get(params, 0));
                    imuData[*imuCount].pitch = json_real_value(json_array_get(params, 1));
                    imuData[*imuCount].yaw = json_real_value(json_array_get(params, 2));
                    imuData[*imuCount].x = json_real_value(json_array_get(params, 3));
                    imuData[*imuCount].y = json_real_value(json_array_get(params, 4));
                    imuData[*imuCount].z = json_real_value(json_array_get(params, 5));
                }
                // 增加 imuCount 计数器，准备存储下一个数据对象
                (*imuCount)++;
            }
        }
    }

    json_decref(root);
}

// IMU数据帧转Pos
Pos IMUDataFrame2Pos(IMUDataFrame *imuData) {
    Pos result = {imuData->x * 1000, imuData->y * 1000, imuData->z * 1000, imuData->roll, imuData->pitch, imuData->yaw};
    return result;
}

// LiDAR数据帧转Pointcloud
void LidarDataFrame2PointCloud(LidarDataFrame *lidarData, PointCloud *lidarPointCloud) {
    lidarPointCloud->ToF_timestamps = lidarData->ToF_timestamps;
    convertToPointCloud(lidarData->ToF_distances, lidarPointCloud->ToF_position);
}


// 运行逻辑
int main()
{
    // 输入数据格式目前是已经做了时间同步的
    LidarDataFrame lidarData[100];        // LiDAR数据帧
    IMUDataFrame imuData[100];            // IMU位姿预测（硬件已做预积分）
    size_t lidarCount = 0, imuCount = 0;
    LidarProcessData("parsed_data.json", lidarData, &lidarCount); // 读取LiDAR数据
    IMUProcessData("parsed_data.json", imuData, &imuCount); // 读取IMU数据

#ifdef DEBUG_PRINT
    printf("Lidar数据:\n");
    for (size_t i = 0; i < lidarCount; ++i)
    {
        printf("Lidar %zu: 时间戳: %d\n", i + 1, lidarData[i].ToF_timestamps);
        printf("距离矩阵:\n");
        for (int row = 0; row < MAX_ROWS; ++row)
        {
            for (int col = 0; col < MAX_COLS; ++col)
            {
                printf("%d ", lidarData[i].ToF_distances[row][col]);
            }
            printf("\n");
        }
    }

    printf("\nIMU数据:\n");
    for (size_t i = 0; i < imuCount; ++i)
    {
        printf("IMU %zu: 时间戳: %d, roll: %.2f, pitch: %.2f, yaw: %.2f, x: %.2f, y: %.2f, z: %.2f\n",
               i + 1, imuData[i].IMU_timestamps,
               imuData[i].roll, imuData[i].pitch, imuData[i].yaw,
               imuData[i].x, imuData[i].y, imuData[i].z);
    }
#endif

#ifdef FILE_PRINT
    // 打开CSV文件用于写入
    FILE *csvFile = fopen("point_cloud_data.csv", "w");
    if (csvFile == NULL)
    {
        perror("无法打开CSV文件");
        return 1;
    }

    fprintf(csvFile, "Timestamp,Row,Col,x,y,z,distance,IMU_x,IMU_y,IMU_z,IMU_roll,IMU_pitch,IMU_yaw,LiDAR_x,LiDAR_y,LiDAR_z,LiDAR_roll,LiDAR_pitch,LiDAR_yaw,EKF_x,EKF_y,EKF_z,EKF_roll,EKF_pitch,EKF_yaw\n");
#endif

    // 用第一帧数据，初始化必要的参数
    EKF_attr ekfattr;
    Pos pos = IMUDataFrame2Pos(&imuData[0]);
    init_ekf(&ekfattr, &pos);
    PointCloud lidarPointCloud;
    LidarDataFrame2PointCloud(&lidarData[0], &lidarPointCloud);
    SLAM_attr slamattr;
    init_slam(&slamattr, pos, &lidarPointCloud);

    #ifdef DEBUG_PRINT
    printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", imuData[0].x, imuData[0].y, imuData[0].z, imuData[0].roll, imuData[0].pitch, imuData[0].yaw);
    printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", ekfattr.pos.x, ekfattr.pos.y, ekfattr.pos.z, ekfattr.pos.roll, ekfattr.pos.pitch, ekfattr.pos.yaw);
    printPointCloud(lidarPointCloud);
    printf("\n");
    printf("framecount: %d \n", slamattr.frameCount);
    printPointCloud(slamattr.globalPointCloud[0]);
    printKDTree(slamattr.kdtree_lastframe[0], 0);
    #endif
    #ifdef FILE_PRINT
        for (int row = 0; row < MAX_ROWS; ++row) {
            for (int col = 0; col < MAX_COLS; ++col) {
                // 输出到CSV文件
                fprintf(csvFile, "%zu,%d,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        lidarData[0].ToF_timestamps,
                        row, col,
                        slamattr.globalPointCloud[0].ToF_position[row][col].x,
                        slamattr.globalPointCloud[0].ToF_position[row][col].y,
                        slamattr.globalPointCloud[0].ToF_position[row][col].z,
                        lidarData[0].ToF_distances[row][col],
                        imuData[0].x * 1000,      // IMU原始x坐标
                        imuData[0].y * 1000,      // IMU原始y坐标
                        imuData[0].z * 1000,      // IMU原始z坐标
                        imuData[0].roll,   // IMU原始roll
                        imuData[0].pitch,  // IMU原始pitch
                        imuData[0].yaw,    // IMU原始yaw
                        pos.x,      // 点云配准得到的x坐标（第一帧用IMU数据代替）
                        pos.y,      // 点云配准得到的y坐标（第一帧用IMU数据代替）
                        pos.z,      // 点云配准得到的z坐标（第一帧用IMU数据代替）
                        pos.roll,   // 点云配准得到的姿态（第一帧用IMU数据代替）
                        pos.pitch,  // 点云配准得到的姿态（第一帧用IMU数据代替）
                        pos.yaw,    // 点云配准得到的姿态（第一帧用IMU数据代替）
                        pos.x,    // EKF融合x坐标
                        pos.y,    // EKF融合y坐标
                        pos.z,    // EKF融合z坐标
                        pos.roll,        // EKF融合roll
                        pos.pitch,       // EKF融合pitch
                        pos.yaw          // EKF融合yaw
                    );
            }
        }
    #endif

    // 之后帧进行SLAM
    Pos last_pos = pos;
    for (size_t i = 1; i < lidarCount && i < imuCount; i++) {
        // ekf预测
        Pos last_IMUpos = IMUDataFrame2Pos(&imuData[i-1]);
        Pos IMUpos = IMUDataFrame2Pos(&imuData[i]);
        ekf_predict(&ekfattr, &last_IMUpos, &IMUpos, 0); // 目前不使用时间戳差分
        Pos pos_predict = ekfattr.pos;

        // slam定位
        LidarDataFrame2PointCloud(&lidarData[i], &lidarPointCloud);
        Pos pos_measure = slam_localization(&slamattr, &lidarPointCloud, pos_predict, last_pos);

        // ekf修正
        ekf_modify(&ekfattr, &pos_measure);
        pos = ekfattr.pos;

        // slam建图
        slam_mapping(&slamattr, pos, &lidarPointCloud);
        last_pos = pos;

        #ifdef FILE_PRINT
        for (int row = 0; row < MAX_ROWS; ++row) {
            for (int col = 0; col < MAX_COLS; ++col) {
                // 输出到CSV文件
                fprintf(csvFile, "%zu,%d,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        lidarData[i].ToF_timestamps,
                        row, col,
                        slamattr.globalPointCloud[i].ToF_position[row][col].x,
                        slamattr.globalPointCloud[i].ToF_position[row][col].y,
                        slamattr.globalPointCloud[i].ToF_position[row][col].z,
                        lidarData[i].ToF_distances[row][col],
                        imuData[i].x * 1000,      // IMU原始x坐标
                        imuData[i].y * 1000,      // IMU原始y坐标
                        imuData[i].z * 1000,      // IMU原始z坐标
                        imuData[i].roll,   // IMU原始roll
                        imuData[i].pitch,  // IMU原始pitch
                        imuData[i].yaw,    // IMU原始yaw
                        pos_measure.x,      // 点云配准得到的x坐标
                        pos_measure.y,      // 点云配准得到的y坐标
                        pos_measure.z,      // 点云配准得到的z坐标
                        pos_measure.roll,   // 点云配准得到的姿态
                        pos_measure.pitch,  // 点云配准得到的姿态
                        pos_measure.yaw,    // 点云配准得到的姿态
                        pos.x,    // EKF融合x坐标
                        pos.y,    // EKF融合y坐标
                        pos.z,    // EKF融合z坐标
                        pos.roll,        // EKF融合roll
                        pos.pitch,       // EKF融合pitch
                        pos.yaw          // EKF融合yaw
                    );
            }
        }
        #endif
    }

#ifdef FILE_PRINT
    fclose(csvFile);
    printf("数据已保存到 point_cloud_data.csv\n");
#endif

    return 0;
}
