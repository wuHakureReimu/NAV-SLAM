
#include <stdio.h>
#include <jansson.h>
#include "mapping.c"
#include "ekf.c"

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

                // 如果有其他字段需要处理，可以在此添加代码
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

// 运行逻辑
int main()
{
    // 输入数据格式目前是已经做了时间同步的
    LidarDataFrame lidarData[100];        // LiDAR数据帧
    IMUDataFrame imuData[100];            // IMU位姿预测（硬件已做预积分）
    PointCloud GlobalpointCloudData[100]; // 全局点云

    size_t lidarCount = 0, imuCount = 0;
    // 读取Lidar数据
    LidarProcessData("parsed_data.json", lidarData, &lidarCount);
    // 读取IMU数据
    IMUProcessData("parsed_data.json", imuData, &imuCount);

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
        printf("IMU %zu: 时间戳: %d, roll: %.6f, pitch: %.6f, yaw: %.6f, x: %.6f, y: %.6f, z: %.6f\n",
               i + 1, imuData[i].IMU_timestamps,
               imuData[i].roll, imuData[i].pitch, imuData[i].yaw,
               imuData[i].x, imuData[i].y, imuData[i].z);
    }
#endif

#ifdef FILE_PRINT
    // 打开CSV文件用于写入
    FILE *csvFile = fopen("point_cloud_data_with_pose.csv", "w");
    if (csvFile == NULL)
    {
        perror("无法打开CSV文件");
        return 1;
    }

    // 表头
    fprintf(csvFile, "Timestamp,Row,Col,x,y,z,distance,IMU_x,IMU_y,IMU_z,Modified_x,Modified_y,Modified_z\n");
#endif

    // 使用EKF紧耦合
    for (size_t i = 1; i < lidarCount + 1; i++)
    {
    }
    // 使用lidarData和imuData进行建图
    for (size_t i = 1; i < lidarCount + 1; i++)
    { // lidarCount + 1
        laserCloudHandler(lidarData[i - 1], imuData[i - 1], &GlobalpointCloudData[i - 1], &i);

#ifdef DEBUG_PRINT
        printf("时间戳: %zu\n", lidarData[i - 1].ToF_timestamps);
        for (int row = 0; row < MAX_ROWS; ++row)
        {
            for (int col = 0; col < MAX_COLS; ++col)
            {
                printf("Point[%d][%d]: x = %.2f, y = %.2f, z = %.2f,distance = %d\n",
                       row, col,
                       GlobalpointCloudData[i - 1].ToF_position[row][col].x,
                       GlobalpointCloudData[i - 1].ToF_position[row][col].y,
                       GlobalpointCloudData[i - 1].ToF_position[row][col].z,
                       lidarData[i - 1].ToF_distances[row][col]);
            }
        }
#endif

#ifdef FILE_PRINT
        for (int row = 0; row < MAX_ROWS; ++row)
        {
            for (int col = 0; col < MAX_COLS; ++col)
            {
                // 输出到CSV文件 - 增加位姿信息
                fprintf(csvFile, "%zu,%d,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        lidarData[i - 1].ToF_timestamps,
                        row, col,
                        GlobalpointCloudData[i - 1].ToF_position[row][col].x,
                        GlobalpointCloudData[i - 1].ToF_position[row][col].y,
                        GlobalpointCloudData[i - 1].ToF_position[row][col].z,
                        lidarData[i - 1].ToF_distances[row][col],
                        currentPose.imu_x,      // IMU原始x坐标
                        currentPose.imu_y,      // IMU原始y坐标
                        currentPose.imu_z,      // IMU原始z坐标
                        currentPose.modified_x, // mapping修正后的x坐标
                        currentPose.modified_y, // mapping修正后的y坐标
                        currentPose.modified_z  // mapping修正后的z坐标
                );
            }
        }
#endif
    }

#ifdef FILE_PRINT
    fclose(csvFile);
    printf("数据已保存到 point_cloud_data_with_pose.csv\n");
#endif

    return 0;
}
