#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jansson.h>
#include <math.h> 
#include <stdbool.h> 

//#define DEBUG_PRINT
#define FILE_PRINT

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

// KDTree节点定义
typedef struct KDNode {
    Point point;        // 存储点数据
    struct KDNode* left;  // 左子树
    struct KDNode* right; // 右子树
} KDNode;

//最近邻点对
typedef struct {
    Point oriPoint; //投影前的点
    Point nearestPoint;
    double distance;//投影后的当前帧点和最近邻点的距离
} NeighborResult;

// 位姿信息结构体
typedef struct {
    double imu_x;      // IMU原始x坐标 (mm)
    double imu_y;      // IMU原始y坐标 (mm)
    double imu_z;      // IMU原始z坐标 (mm)
    double modified_x; // mapping修正后的x坐标 (mm)
    double modified_y; // mapping修正后的y坐标 (mm)
    double modified_z; // mapping修正后的z坐标 (mm)
} PoseInfo;

//3个自由度的位姿变化：xyz 3个方向的位移变化
KDNode* lastKdTreeRoot[MAX_ROWS] = { NULL };  // 用来存储上一帧每一行的 KD-Tree 根节点
PointCloud* lastGlobalPointCloudData = NULL;  // 上一帧的全局点云数据
IMUDataFrame lastModifiedPosition ;  // 上一帧的修正后IMU位置

//是否是第一帧
bool isFirstFrame = true;

// 全局变量存储当前帧的位姿信息
PoseInfo currentPose;

//-------------------------------------------------------------------------------------------------------------------------

// 读取Lidar数据函数
void LidarProcessData(const char *filename, LidarDataFrame *lidarData, size_t *lidarCount) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("无法打开文件");
        return;
    }

    json_error_t error;
    json_t *root = json_loadf(file, 0, &error);
    fclose(file);
    if (!root) {
        printf("JSON解析错误: %s\n", error.text);
        return;
    }

    // 检查root是否是一个数组，且数组中有元素
    if (json_is_array(root) && json_array_size(root) > 0) {
        size_t array_size = json_array_size(root);

        // 遍历整个数组中的每个数据对象
        for (size_t i = 0; i < array_size; i++) {
            json_t *obj = json_array_get(root, i);  // 获取数组中的第 i 个对象
            if (json_is_object(obj)) {

                // 读取 "time_main" 字段
                json_t *time_main = json_object_get(obj, "time_main");
                if (json_is_integer(time_main)) {
                    lidarData[*lidarCount].ToF_timestamps = json_integer_value(time_main);
                }

                // 解析 "distance" 数组并存入矩阵
                json_t *distance = json_object_get(obj, "distance");
                if (json_is_array(distance)) {
                    size_t index = 0;
                    json_t *value;
                    json_array_foreach(distance, index, value) {
                        if (json_is_integer(value) && index < MAX_ROWS * MAX_COLS) {
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
void IMUProcessData(const char *filename, IMUDataFrame *imuData, size_t *imuCount) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("无法打开文件");
        return;
    }

    json_error_t error;
    json_t *root = json_loadf(file, 0, &error);
    fclose(file);
    if (!root) {
        printf("JSON解析错误: %s\n", error.text);
        return;
    }

    if (json_is_array(root) && json_array_size(root) > 0) {
        size_t array_size = json_array_size(root);

        // 遍历整个数组中的每个数据对象
        for (size_t i = 0; i < array_size; i++) {
            json_t *obj = json_array_get(root, i);  // 获取数组中的第 i 个对象
            if (json_is_object(obj)) {

                // 解析 "time_main" 字段
                json_t *time_main = json_object_get(obj, "time_main");
                if (json_is_integer(time_main)) {
                    imuData[*imuCount].IMU_timestamps = json_integer_value(time_main);
                }

                // 解析 "params" 数组
                json_t *params = json_object_get(obj, "params");
                if (json_is_array(params) && json_array_size(params) == 6) {
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

// 将 lidar 数据帧的 ToF_distances 转换为 PointCloudData 的 3D 坐标
void convertToPointCloud(int distances[MAX_ROWS][MAX_COLS], Point pointCloud[MAX_ROWS][MAX_COLS]) {
    // 定义水平和垂直视场角（度）
    const double fov_h = 45.0;  // 水平视场角
    const double fov_v = 45.0;  // 垂直视场角

    // 计算每个方向上的角度步长（度）
    const double theta_step_deg = fov_h / (MAX_COLS - 1);  // 水平角度步长
    const double phi_step_deg = fov_v / (MAX_ROWS - 1);    // 垂直角度步长

    // 转换步长为弧度
    const double theta_step_rad = theta_step_deg * M_PI / 180.0;  // 水平角度步长（弧度）
    const double phi_step_rad = phi_step_deg * M_PI / 180.0;    // 垂直角度步长（弧度）

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

// 计算曲率
int computeCurvature(int i, int row[MAX_COLS]) {
    int left = (i > 0) ? abs(row[i] - row[i - 1]) : 0;
    int right = (i < MAX_COLS - 1) ? abs(row[i] - row[i + 1]) : 0;
    if((i <= 0)||(i >= MAX_COLS - 1)) return 0;
    return left + right;
}

// 扁平化PointCloud并筛选出特征点
void flattenPoints(Point rowPoints[MAX_COLS], int FeatureMatrix[MAX_COLS], Point *flattenedPoints, size_t *numPoints) {
    *numPoints = 0;

    for (int i = 0; i < MAX_COLS; ++i) {
        if (FeatureMatrix[i] == 1) {
            flattenedPoints[*numPoints] = rowPoints[i];
            (*numPoints)++;
        }
    }

    #ifdef DEBUG_PRINT
        printf("扁平化后的特征点：\n");
        for (size_t i = 0; i < *numPoints; ++i) {
            printf("Point[%zu]: x = %.2f, y = %.2f, z = %.2f\n", i, flattenedPoints[i].x, flattenedPoints[i].y, flattenedPoints[i].z);
        }
    #endif
}

// 获取分割轴（根据深度进行递归）
int getAxis(int depth) {
    return depth % 3;  // 0: x轴, 1: y轴, 2: z轴
}

// 计算欧几里得距离
double euclideanDistance(Point p1, Point p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

// 构建KD-Tree的递归函数
KDNode* buildKDTree(Point *points, size_t numPoints, int depth) {
    if (numPoints == 0) {
        return NULL;
    }

    // 选择分割轴
    int axis = getAxis(depth);

    // 按照当前轴排序
    for (size_t i = 0; i < numPoints - 1; ++i) {
        for (size_t j = i + 1; j < numPoints; ++j) {
            double compare = 0;
            switch (axis) {
                case 0: compare = points[i].x - points[j].x;break;  // x轴
                case 1: compare = points[i].y - points[j].y;break;  // y轴
                case 2: compare = points[i].z - points[j].z;break;  // z轴
            }
            if (compare > 0) {
                Point temp = points[i];
                points[i] = points[j];
                points[j] = temp;
            }
        }
    }

    
    // 选择中位数作为节点
    size_t median = numPoints / 2;
    KDNode* node = (KDNode*)malloc(sizeof(KDNode));
    node->point = points[median];

    // 构建左右子树
    node->left = buildKDTree(points, median, depth + 1);
    node->right = buildKDTree(points + median + 1, numPoints - median - 1, depth + 1);

    return node;
}

// 递归输出KD-Tree节点
void printKDTree(KDNode* node, int depth) {
    if (node == NULL) {
        return;
    }

    // 打印当前节点的坐标
    printf("深度 %d: Point(x=%.2f, y=%.2f, z=%.2f)\n", depth, node->point.x, node->point.y, node->point.z);

    // 递归打印左子树和右子树
    printKDTree(node->left, depth + 1);
    printKDTree(node->right, depth + 1);
}

// 计算位移的函数
void computeDisplacement(IMUDataFrame* imuData, IMUDataFrame* lastModifiedPosition, double transform[3]) {
    // 计算位移: imuData的坐标与上一帧的位置进行差值
    transform[0] = imuData->x *1000 - lastModifiedPosition->x ;  // 计算x轴位移
    transform[1] = imuData->y *1000 - lastModifiedPosition->y ;  // 计算y轴位移
    transform[2] = imuData->z *1000 - lastModifiedPosition->z ;  // 计算z轴位移

    // 打印位移以进行验证
    #ifdef DEBUG_PRINT
        printf("x: %.6f, y: %.6f, z: %.6f\n", transform[0], transform[1], transform[2]);
    #endif
}

// 根据位移 transform 进行坐标映射
void mapCoordinatesToLastFrame(PointCloud* globalPointCloudData, double transform[3], PointCloud* positionInLastFrame) {
    // 遍历每一行和每一列
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            // 将当前坐标减去transform（即计算位置在上一帧的坐标）
            positionInLastFrame->ToF_position[row][col].x = globalPointCloudData->ToF_position[row][col].x - transform[0];
            positionInLastFrame->ToF_position[row][col].y = globalPointCloudData->ToF_position[row][col].y - transform[1];
            positionInLastFrame->ToF_position[row][col].z = globalPointCloudData->ToF_position[row][col].z - transform[2];
        }
    }
}

// 最近邻搜索函数
void nearestNeighborSearch(KDNode* root, Point* target, Point* result, double bestDist, int depth) {
    if (root == NULL) return;

    // 计算当前节点到目标点的距离
    double dist = euclideanDistance(root->point, *target);
    if (dist < bestDist) {
        bestDist = dist;
        *result = root->point;  // 假设此时找到了最近邻点
    }

    // 确定当前轴
    int axis = getAxis(depth);

    // 根据目标点的坐标判断递归搜索哪个子树
    KDNode* nextBranch = NULL;
    KDNode* oppositeBranch = NULL;

    if ((axis == 0 && target->x < root->point.x) ||
        (axis == 1 && target->y < root->point.y) ||
        (axis == 2 && target->z < root->point.z)) {
        nextBranch = root->left;
        oppositeBranch = root->right;
    } else {
        nextBranch = root->right;
        oppositeBranch = root->left;
    }

    // 递归搜索下一个子树
    nearestNeighborSearch(nextBranch, target, result, bestDist, depth + 1);

    // 如果有可能，检查另一个子树
    if (fabs((axis == 0 ? target->x - root->point.x :
               axis == 1 ? target->y - root->point.y :
                           target->z - root->point.z)) < bestDist) {
        nearestNeighborSearch(oppositeBranch, target, result, bestDist, depth + 1);
    }
}

void laserCloudHandler(LidarDataFrame lidarDataFrame, IMUDataFrame imuData, PointCloud *globalPointCloudData,size_t *frameCount) {
    IMUDataFrame CurModifiedPosition; // pm(i) 修正后的imu全局位置
    PointCloud PointCloudData_TOF; //相对于TOF的3D坐标
    double transform[3] = {0}; //3个自由度的变换

    PointCloudData_TOF.ToF_timestamps = lidarDataFrame.ToF_timestamps;
    globalPointCloudData->ToF_timestamps = lidarDataFrame.ToF_timestamps;

    // 设置当前位姿信息中的IMU原始坐标（转换为毫米）
    currentPose.imu_x = imuData.x * 1000;
    currentPose.imu_y = imuData.y * 1000;
    currentPose.imu_z = imuData.z * 1000;

    //深度图转换为相对于tof传感器的相对坐标
    convertToPointCloud(lidarDataFrame.ToF_distances, PointCloudData_TOF.ToF_position);

    #ifdef DEBUG_PRINT
        printf("时间戳: %zu\n", *frameCount);
        for (int row = 0; row < MAX_ROWS; ++row) {
            for (int col = 0; col < MAX_COLS; ++col) {
                printf("Point[%d][%d]: x = %.2f, y = %.2f, z = %.2f,distance = %d\n",
                    row, col,
                    PointCloudData_TOF.ToF_position[row][col].x,
                    PointCloudData_TOF.ToF_position[row][col].y,
                    PointCloudData_TOF.ToF_position[row][col].z,
                    lidarDataFrame.ToF_distances[row][col]);
            }
        }
    #endif
    

    //根据imu提供的全局无人机坐标计算数据点的3维绝对坐标
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            globalPointCloudData->ToF_position[row][col].x =  (imuData.x * 1000) +  PointCloudData_TOF.ToF_position[row][col].x ;
            globalPointCloudData->ToF_position[row][col].y =  (imuData.y * 1000) +  PointCloudData_TOF.ToF_position[row][col].y ;
            globalPointCloudData->ToF_position[row][col].z =  (imuData.z * 1000) +  PointCloudData_TOF.ToF_position[row][col].z ;
        }
    }

    //特征提取
    int curvatureMatrix[MAX_ROWS][MAX_COLS];//曲率矩阵
    int FeatureMatrix[MAX_ROWS][MAX_COLS] = { 0 }; // 初始化特征矩阵
    
    // 计算曲率矩阵
    for (int i = 0; i < MAX_ROWS; ++i) {
        for (int j = 0; j < MAX_COLS; ++j) {
            curvatureMatrix[i][j] = computeCurvature(j, lidarDataFrame.ToF_distances[i]);
        }
    }
    
    #ifdef DEBUG_PRINT
        printf("时间戳: %zu\n", *frameCount);
        for (int i = 0; i < MAX_ROWS; ++i) {
            for (int j = 0; j < MAX_COLS; ++j) {
                printf("%d ", curvatureMatrix[i][j]);
            }
            printf("\n");
        }
    #endif

    // 特征提取
    for (int i = 0; i < MAX_ROWS; ++i) {
        for (int j = 0; j < MAX_COLS; ++j) {
            if(curvatureMatrix[i][j]>500 && lidarDataFrame.ToF_distances[i][j]<1000){//前景的边缘
                FeatureMatrix[i][j] = 1;
            }
        
        }
    }

    #ifdef DEBUG_PRINT
        printf("时间戳: %zu\n", *frameCount);
        for (int i = 0; i < MAX_ROWS; ++i) {
            for (int j = 0; j < MAX_COLS; ++j) {
                printf("%d", FeatureMatrix[i][j]);
            }
            printf("\n");
        }
    #endif

    //loam 的场景重建 mapping

    // 扁平化每一行的特征点并构建KD-Tree
    KDNode* kdTreeRoot[MAX_ROWS];
    
    printf("时间戳: %zu\n", *frameCount);
    //第一帧
    if(isFirstFrame){
        isFirstFrame = false;
        
        //构建kd-tree，仅仅包含特征点
        for (int row = 0; row < MAX_ROWS; ++row) {
            Point flattenedPoints[MAX_COLS];
            size_t numPoints = 0;
            flattenPoints(globalPointCloudData->ToF_position[row], FeatureMatrix[row], flattenedPoints, &numPoints);
            kdTreeRoot[row] = buildKDTree(flattenedPoints, numPoints, 0); // 对每一行数据单独构建KD-Tree
        }

        //保存上一帧
        for (int row = 0; row < MAX_ROWS; ++row) {
            // 保存上一帧每一行的KD-Tree根节点
            lastKdTreeRoot[row] = kdTreeRoot[row];
        }
        lastGlobalPointCloudData = globalPointCloudData;  // 将当前的全局点云数据赋值给lastGlobalPointCloudData

        lastModifiedPosition.IMU_timestamps = imuData.IMU_timestamps;  // 将当前的IMU数据赋值给lastIMUdata
        lastModifiedPosition.x = imuData.x * 1000;
        lastModifiedPosition.y = imuData.y * 1000;
        lastModifiedPosition.z = imuData.z * 1000;

        // 第一帧时，修正后的坐标等于IMU原始坐标
        currentPose.modified_x = currentPose.imu_x;
        currentPose.modified_y = currentPose.imu_y;
        currentPose.modified_z = currentPose.imu_z;

        return  ; // Pimu1 = Pm1 imu提供的坐标直接作为全局坐标，不修正
    }

    //第2帧及以后
    //1. Tr(i) = Pimu(i) - Pm(i-1)  初始位移 imu提供的i帧坐标 - 修正后的 i-1 帧坐标
    computeDisplacement(&imuData, &lastModifiedPosition, transform);  // 计算位移，transform单位：mm

    //2. Pt(i) = Tr(i) + Pimu(i)  第 i 帧的点按照这个位移反映射回 i - 1帧的位置 Pt(i)
    PointCloud positionInLastFrame;
    // 根据 transform 对坐标进行映射，映射回上一帧
    mapCoordinatesToLastFrame(globalPointCloudData, transform, &positionInLastFrame); // positionInLastFrame单位：mm
    
    #ifdef DEBUG_PRINT
            // 打印 globalPointCloudData
            printf("时间戳: %d\n", imuData.IMU_timestamps);
            printf("当前帧的 globalPointCloudData:\n");
            for (int row = 0; row < MAX_ROWS; ++row) {
                for (int col = 0; col < MAX_COLS; ++col) {
                    printf("globalPointCloudData[%d][%d]: x = %.2f, y = %.2f, z = %.2f\n",
                        row, col,
                        globalPointCloudData->ToF_position[row][col].x,
                        globalPointCloudData->ToF_position[row][col].y,
                        globalPointCloudData->ToF_position[row][col].z);
                }
            }
        
            // 打印 positionInLastFrame
            printf("映射后的 positionInLastFrame:\n");
            for (int row = 0; row < MAX_ROWS; ++row) {
                for (int col = 0; col < MAX_COLS; ++col) {
                    if(FeatureMatrix[row][col] == 1){
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
    double learningRate = 0.8; // 设置学习率
    double tolerance = 1e-6;     // 收敛容忍度
    double previousTotalError = 0;
    double totalError = 0;

    for (int iter = 0; iter < 100; ++iter) {
        // 3. 构造几何关系，误差函数
        // 3.1 使用 kd-tree（lastKdTreeRoot），找到 映射后的i帧中所有特征点的 与其在同一行的 1 个i-1 帧中的最近邻点
        if (iter % 100 == 0) {
            int flag = 0; 
            for (int row = 0; row < MAX_ROWS; ++row) {

                for (int col = 0; col < MAX_COLS; ++col) {
                    if (FeatureMatrix[row][col] == 1) {  // 只处理特征点

                    //最近邻查找
                        Point currentPoint = positionInLastFrame.ToF_position[row][col];
                        Point CurnearestPoint = currentPoint;
                        double bestDist = INFINITY;  // 初始设为一个非常大的值
                        nearestNeighborSearch(lastKdTreeRoot[row], &currentPoint, &CurnearestPoint, bestDist, 0);  // 修改为针对每一行的KDTree

                    //放入最近邻数组
                        //每一行的第一个点
                        if(flag == CPcount){
                            result[CPcount].oriPoint = globalPointCloudData->ToF_position[row][col];
                            result[CPcount].nearestPoint = CurnearestPoint;
                            result[CPcount].distance = euclideanDistance(currentPoint,CurnearestPoint);
                            CPcount++;
                            continue;
                        }

                        //每一行的其他点
                        for(int i = flag; i < CPcount; i++){
                            if((result[i].nearestPoint.x == CurnearestPoint.x)&&
                                (result[i].nearestPoint.y == CurnearestPoint.y)&&
                                (result[i].nearestPoint.z == CurnearestPoint.z)){
                                    if(result[i].distance > euclideanDistance(currentPoint,CurnearestPoint)){
                                        result[i].oriPoint = globalPointCloudData->ToF_position[row][col];
                                        result[i].nearestPoint = CurnearestPoint;
                                        result[i].distance = euclideanDistance(currentPoint,CurnearestPoint);
                                        break;
                                    }
                                    else{
                                        break;
                                    }
                                }
                                else{
                                    result[CPcount].oriPoint = globalPointCloudData->ToF_position[row][col];
                                    result[CPcount].nearestPoint = CurnearestPoint;
                                    result[CPcount].distance = euclideanDistance(currentPoint,CurnearestPoint);
                                    CPcount++;
                                }
                        }
                        
                    }//result单位：mm
                }
                flag =  CPcount;
            }//end for

            #ifdef DEBUG_PRINT
                printf("时间戳: %d\n", imuData.IMU_timestamps);
                printf("时间戳: %zu\n", *frameCount);
                for(int i = 0 ; i < CPcount ; i++){
                    printf("Result %d:\n", i);
                    printf("  Original Point: (%.2f, %.2f, %.2f)\n", result[i].oriPoint.x, result[i].oriPoint.y, result[i].oriPoint.z);
                    printf("  project Point: (%.2f, %.2f, %.2f)\n", result[i].oriPoint.x - transform[0], result[i].oriPoint.y- transform[1], result[i].oriPoint.z - transform[2]);
                    printf("  Nearest Point: (%.2f, %.2f, %.2f)\n", result[i].nearestPoint.x, result[i].nearestPoint.y, result[i].nearestPoint.z);
                    printf("ErrDistance = %.2f mm\n", result[i].distance);
                }
            #endif
        }


        //3.2 计算误差 d （具体的定义方式有待商榷）
        double ErrDistance[100];
        for(int i = 0 ; i < CPcount ; i++){
            ErrDistance[i] = sqrt(pow((result[i].oriPoint.x - transform[0]) - result[i].nearestPoint.x, 2) + 
                                pow((result[i].oriPoint.y - transform[1]) - result[i].nearestPoint.y, 2) +
                                pow((result[i].oriPoint.z - transform[2]) - result[i].nearestPoint.z,2));
        }

        #ifdef DEBUG_PRINT
            printf("时间戳: %d\n", imuData.IMU_timestamps);
            for (int i = 0; i < CPcount; i++) {
                printf("ErrDistance[%d] = %.2f mm\n", i, ErrDistance[i]);
            }
        #endif

        // 更稳健的梯度下降实现
        double gradient[3] = {0.0, 0.0, 0.0};
        totalError = 0;

        // 使用平方误差，避免开方运算
        for (int i = 0; i < CPcount; i++) {
            double dx = (result[i].oriPoint.x - transform[0]) - result[i].nearestPoint.x;
            double dy = (result[i].oriPoint.y - transform[1]) - result[i].nearestPoint.y;
            double dz = (result[i].oriPoint.z - transform[2]) - result[i].nearestPoint.z;
            
            double dist_sq = dx*dx + dy*dy + dz*dz;
            totalError += dist_sq;
            
            // 梯度计算：E = 1/2 * sum(dist^2)，dE/dtransform = -sum(dx, dy, dz)
            gradient[0] -= dx;  // 负梯度方向
            gradient[1] -= dy;
            gradient[2] -= dz;
        }

        // 判断收敛
        if (fabs(totalError - previousTotalError) < tolerance) {
            printf("收敛，停止迭代！\n");
            break;
        }
        previousTotalError = totalError;

        // 自适应学习率 - 根据梯度大小调整
        double grad_norm = sqrt(gradient[0]*gradient[0] + gradient[1]*gradient[1] + gradient[2]*gradient[2]);
        if (grad_norm > 1e-6) {
            double adaptive_lr = learningRate / grad_norm;  // 归一化
            
            transform[0] += adaptive_lr * gradient[0];
            transform[1] += adaptive_lr * gradient[1];
            transform[2] += adaptive_lr * gradient[2];
        }

        printf("Iteration %d, Total Error: %.6f, Gradient Norm: %.6f\n", iter, totalError, grad_norm);
    }

    //计算pm(i) = pm(i-1) + Tr(i) 修正后的 i 帧无人机位置 = 修正后的 i-1 帧无人机位置 + 迭代优化后的位移
    CurModifiedPosition.IMU_timestamps = imuData.IMU_timestamps;
    CurModifiedPosition.x = lastModifiedPosition.x + transform[0] ;
    CurModifiedPosition.y = lastModifiedPosition.y + transform[1] ;
    CurModifiedPosition.z = lastModifiedPosition.z + transform[2] ;

    // 设置当前位姿信息中的修正后坐标
    currentPose.modified_x = CurModifiedPosition.x;
    currentPose.modified_y = CurModifiedPosition.y;
    currentPose.modified_z = CurModifiedPosition.z;

    //计算修正后的点云坐标
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            globalPointCloudData->ToF_position[row][col].x =  (CurModifiedPosition.x) +  PointCloudData_TOF.ToF_position[row][col].x ;
            globalPointCloudData->ToF_position[row][col].y =  (CurModifiedPosition.y) +  PointCloudData_TOF.ToF_position[row][col].y ;
            globalPointCloudData->ToF_position[row][col].z =  (CurModifiedPosition.z) +  PointCloudData_TOF.ToF_position[row][col].z ;
        }
    }

    //构建kd-tree供下一帧使用，仅仅包含特征点
    for (int row = 0; row < MAX_ROWS; ++row) {
        Point flattenedPoints[MAX_COLS];
        size_t numPoints = 0;
        flattenPoints(globalPointCloudData->ToF_position[row], FeatureMatrix[row], flattenedPoints, &numPoints);
        kdTreeRoot[row] = buildKDTree(flattenedPoints, numPoints, 0); // 对每一行数据单独构建KD-Tree
    }

    //保存上一帧
    for (int row = 0; row < MAX_ROWS; ++row) {
        // 保存上一帧每一行的KD-Tree根节点
        lastKdTreeRoot[row] = kdTreeRoot[row];
    }

    lastGlobalPointCloudData = globalPointCloudData;  // 将当前的全局点云数据赋值给lastGlobalPointCloudData
    lastModifiedPosition = CurModifiedPosition;
}

int main() {
    LidarDataFrame lidarData[100];  // 假设最多100个Lidar数据帧
    IMUDataFrame imuData[100];  // 假设最多100个IMU数据帧
    PointCloud GlobalpointCloudData[100]; //每个数据帧的3d点云，全局坐标系下

    size_t lidarCount = 0, imuCount = 0;

    // 读取Lidar数据
    LidarProcessData("parsed_data_copy.json", lidarData, &lidarCount);
    
    // 读取IMU数据
    IMUProcessData("parsed_data_copy.json", imuData, &imuCount);

    #ifdef DEBUG_PRINT
        printf("Lidar数据:\n");
        for (size_t i = 0; i < lidarCount; ++i) {
            printf("Lidar %zu: 时间戳: %d\n", i + 1, lidarData[i].ToF_timestamps);
            printf("距离矩阵:\n");
            for (int row = 0; row < MAX_ROWS; ++row) {
                for (int col = 0; col < MAX_COLS; ++col) {
                    printf("%d ", lidarData[i].ToF_distances[row][col]);
                }
                printf("\n");
            }
        }

        printf("\nIMU数据:\n");
        for (size_t i = 0; i < imuCount; ++i) {
            printf("IMU %zu: 时间戳: %d, x: %.6f, y: %.6f, z: %.6f\n", 
                i + 1, imuData[i].IMU_timestamps,
                imuData[i].x, imuData[i].y, imuData[i].z);
        }
    #endif

    #ifdef FILE_PRINT
        // 打开CSV文件用于写入
        FILE *csvFile = fopen("point_cloud_data_with_pose.csv", "w");
        if (csvFile == NULL) {
            perror("无法打开CSV文件");
            return 1;
        }

        // 写入CSV文件的表头 - 增加位姿信息列
        fprintf(csvFile, "Timestamp,Row,Col,x,y,z,distance,IMU_x,IMU_y,IMU_z,Modified_x,Modified_y,Modified_z\n");
    #endif

    //使用lidarData和imuData进行建图
    for (size_t i = 1; i < lidarCount + 1 ; i++) {//lidarCount + 1
        laserCloudHandler(lidarData[i-1], imuData[i-1], &GlobalpointCloudData[i-1],&i);

        #ifdef DEBUG_PRINT
            printf("时间戳: %zu\n",lidarData[i-1].ToF_timestamps );
            for (int row = 0; row < MAX_ROWS; ++row) {
                for (int col = 0; col < MAX_COLS; ++col) {
                    printf("Point[%d][%d]: x = %.2f, y = %.2f, z = %.2f,distance = %d\n",
                        row, col,
                        GlobalpointCloudData[i-1].ToF_position[row][col].x,
                        GlobalpointCloudData[i-1].ToF_position[row][col].y,
                        GlobalpointCloudData[i-1].ToF_position[row][col].z,
                        lidarData[i-1].ToF_distances[row][col]);
                }
            }
        #endif

        #ifdef FILE_PRINT
            for (int row = 0; row < MAX_ROWS; ++row) {
                    for (int col = 0; col < MAX_COLS; ++col) {
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
