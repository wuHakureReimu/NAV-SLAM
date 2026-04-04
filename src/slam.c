#include <stdio.h>
#include <stddef.h>
#include "slam.h"
#include "math.h"
#include "kdtree.h"
#include "pointcloud.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)
// #define DEBUG_PRINT
#define FEATURE_DEBUG_PRINT

// 计算曲率以及特征（仅边缘点）提取
void extract_feature(PointCloud *lidarPointCloud, int edge_feature[MAX_ROWS][MAX_COLS], int plane_feature[MAX_ROWS][MAX_COLS])
{
    const int smooth_window = 2;         // 平滑窗口大小，可调整
    const double edge_threshold = 0.10;  // 曲率阈值
    const double plane_threshold = 0.10; // 平面点阈值
    const int max_edge_per_row = 2;
    const int max_plane_per_row = 2;

    // 遍历每个点（跳过边缘的点）
    for (int i = 0; i < MAX_ROWS; i++)
    {
        double curvature[MAX_COLS] = {0.0}; // 存储每个点的曲率值
        for (int j = smooth_window; j < MAX_COLS - smooth_window; j++)
        {
            // 获取当前点
            Point current_point = lidarPointCloud->ToF_position[i][j];

            // 计算邻域点的距离加权曲率
            double sum_dist = 0.0;
            int count = 0;
            // 计算前向和后向邻域点的距离
            for (int k = -smooth_window; k <= smooth_window; k++)
            {
                if (k == 0)
                    continue;

                Point neighbor_point = lidarPointCloud->ToF_position[i][j + k];
                double dx = current_point.x - neighbor_point.x;
                double dy = current_point.y - neighbor_point.y;
                double dz = current_point.z - neighbor_point.z;

                double dist_sq = dx * dx + dy * dy + dz * dz;
                sum_dist += sqrt(dist_sq);
                count++;
            }
            // 计算平均距离
            double avg_dist = (count > 0) ? sum_dist / count : 0;

            // 计算曲率
            double curv = 0.0;
            // 基于距离方差计算
            if (count > 0 && avg_dist > 0)
            {
                double sum_var = 0.0;
                for (int k = -smooth_window; k <= smooth_window; k++)
                {
                    if (k == 0)
                        continue;
                    Point neighbor_point = lidarPointCloud->ToF_position[i][j + k];
                    double dx = current_point.x - neighbor_point.x;
                    double dy = current_point.y - neighbor_point.y;
                    double dz = current_point.z - neighbor_point.z;
                    double dist = sqrt(dx * dx + dy * dy + dz * dz);

                    sum_var += (dist - avg_dist) * (dist - avg_dist);
                }
                curv = sum_var / count / (avg_dist * avg_dist + 1e-6f);
            }
            curvature[j] = curv;
        }
        // 按照曲率排序选 edge 和 plane 特征点
        int suppressed_edge[MAX_COLS] = {0};
        for (int n = 0; n < max_edge_per_row; n++)
        {
            int best_col = -1;
            double best_curv = edge_threshold;
            for (int j = smooth_window; j < MAX_COLS - smooth_window; j++)
            {
                if (suppressed_edge[j])
                    continue; // 跳过已抑制的点
                if (curvature[j] > best_curv)
                {
                    best_curv = curvature[j];
                    best_col = j;
                }
            }

            if (best_col != -1)
            {
                edge_feature[i][best_col] = 1;
                // 抑制相邻点
                for (int t = best_col - 1; t <= best_col + 1; t++)
                {
                    if (t >= 0 && t < MAX_COLS)
                        suppressed_edge[t] = 1;
                }
            }
        }
        int suppressed_plane[MAX_COLS] = {0};
        for (int j = 0; j < MAX_COLS; j++)
        {
            if (edge_feature[i][j] == 1)
                suppressed_plane[j] = 1; // 如果是边缘特征点，则不考虑为面特征点
        }
        for (int n = 0; n < max_plane_per_row; n++)
        {
            int best_col = -1;
            double best_curv = 1e9;

            for (int j = smooth_window; j < MAX_COLS - smooth_window; j++)
            {
                if (suppressed_plane[j])
                    continue;

                if (curvature[j] < best_curv)
                {
                    best_curv = curvature[j];
                    best_col = j;
                }
            }

            if (best_col != -1 && best_curv < plane_threshold)
            {
                plane_feature[i][best_col] = 1;

                for (int t = best_col - 1; t <= best_col + 1; t++)
                {
                    if (t >= 0 && t < MAX_COLS)
                        suppressed_plane[t] = 1;
                }
            }
        }
    }

    int edge_count = 0, plane_count = 0;
    for (int i = 0; i < MAX_ROWS; i++)
    {
        for (int j = 0; j < MAX_COLS; j++)
        {
            if (edge_feature[i][j])
                edge_count++;
            if (plane_feature[i][j])
                plane_count++;
        }
    }
    printf("edge_count = %d, plane_count = %d\n", edge_count, plane_count);
}

// 从雷达点云提取特征点


// 辅助函数：扁平化PointCloud并筛选出特征点
void flattenPoints(Point rowPoints[MAX_COLS], int rowFeature[MAX_COLS], Point flattenedPoints[MAX_COLS], size_t *numPoints)
{
    *numPoints = 0;
    for (int i = 0; i < MAX_COLS; ++i)
    {
        if (rowFeature[i] == 1)
        {
            flattenedPoints[*numPoints] = rowPoints[i];
            (*numPoints)++;
        }
    }
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

// 辅助函数：把“基准位姿下的全局点”按 6DoF 增量变换到当前估计位姿
// delta[0..2]：平移增量（mm），delta[3..5]：旋转增量（rad）
static void applyIncrementalPoseToPoint(const Point *basePoint,
                                        const Pos *basePose,
                                        const double delta[6],
                                        Point *outPoint)
{
    double Rinc[3][3];
    getRotationMatrix(delta[3], delta[4], delta[5], Rinc);

    // 基准位姿下，相对于 basePose 平移原点的向量
    double sx = basePoint->x - basePose->x;
    double sy = basePoint->y - basePose->y;
    double sz = basePoint->z - basePose->z;

    // 对该向量施加小旋转
    double rx = Rinc[0][0] * sx + Rinc[0][1] * sy + Rinc[0][2] * sz;
    double ry = Rinc[1][0] * sx + Rinc[1][1] * sy + Rinc[1][2] * sz;
    double rz = Rinc[2][0] * sx + Rinc[2][1] * sy + Rinc[2][2] * sz;

    // 再施加平移增量，得到当前估计位姿下的全局点
    outPoint->x = basePose->x + delta[0] + rx;
    outPoint->y = basePose->y + delta[1] + ry;
    outPoint->z = basePose->z + delta[2] + rz;
}

// 辅助函数：根据 6DoF 增量，把当前帧所有点映射到当前估计位姿下
void mapCoordinatesToLastFrame(PointCloud *globalPointCloudData, const Pos *basePose,
                               const double delta[6], PointCloud *positionInLastFrame)
{
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
            applyIncrementalPoseToPoint(&globalPointCloudData->ToF_position[row][col],
                                        basePose, delta,
                                        &positionInLastFrame->ToF_position[row][col]);
        }
    }
}

// 6x6 线性方程组求解（带主元高斯消元）
// 输入：A x = b
// 输出：x
// 成功返回 1，失败返回 0
static int solveLinearSystem6x6(double A[6][6], double b[6], double x[6])
{
    double aug[6][7];

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
            aug[i][j] = A[i][j];
        aug[i][6] = b[i];
    }

    for (int col = 0; col < 6; ++col)
    {
        int pivot = col;
        double maxAbs = fabs(aug[col][col]);

        for (int row = col + 1; row < 6; ++row)
        {
            double v = fabs(aug[row][col]);
            if (v > maxAbs)
            {
                maxAbs = v;
                pivot = row;
            }
        }

        if (maxAbs < 1e-9)
            return 0;

        if (pivot != col)
        {
            for (int j = col; j < 7; ++j)
            {
                double tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }

        double div = aug[col][col];
        for (int j = col; j < 7; ++j)
            aug[col][j] /= div;

        for (int row = 0; row < 6; ++row)
        {
            if (row == col)
                continue;

            double factor = aug[row][col];
            for (int j = col; j < 7; ++j)
                aug[row][j] -= factor * aug[col][j];
        }
    }

    for (int i = 0; i < 6; ++i)
        x[i] = aug[i][6];

    return 1;
}

static double huberWeightFromSquared(double sq_err, double delta)
{
    double err = sqrt(sq_err);
    if (err <= delta)
        return 1.0;
    return delta / (err + 1e-12);
}

static void clampPoseStep(double step[6], double max_step_t, double max_step_r)
{
    double trans_norm = sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]);
    if (trans_norm > max_step_t && trans_norm > 1e-12)
    {
        double s = max_step_t / trans_norm;
        step[0] *= s;
        step[1] *= s;
        step[2] *= s;
    }

    double rot_norm = sqrt(step[3] * step[3] + step[4] * step[4] + step[5] * step[5]);
    if (rot_norm > max_step_r && rot_norm > 1e-12)
    {
        double s = max_step_r / rot_norm;
        step[3] *= s;
        step[4] *= s;
        step[5] *= s;
    }
}

// 辅助函数：在当前行附近搜索两个可用的plane邻居行
static int findTwoNearestPlaneNeighbors(SLAM_attr *attr, Point *anchorPoint, int centerRow, int searchRadius,
                                        int *rowA, Point *pointA, double *distA, int *rowC, Point *pointC, double *distC)
{
    *rowA = -1;
    *rowC = -1;
    *distA = INFINITY;
    *distC = INFINITY;

    for (int offset = 1; offset <= searchRadius; ++offset)
    {
        int candidateRows[2] = {centerRow - offset, centerRow + offset};

        for (int k = 0; k < 2; ++k)
        {
            int r = candidateRows[k];
            if (r < 0 || r >= MAX_ROWS)
                continue;
            if (attr->kdtree_plane_lastframe[r] == NULL)
                continue;

            Point candidatePoint;
            double candidateDist = INFINITY;
            nearestNeighborSearch(attr->kdtree_plane_lastframe[r],
                                  anchorPoint,
                                  &candidatePoint,
                                  &candidateDist,
                                  0);

            if (candidateDist < *distA)
            {
                *distC = *distA;
                *pointC = *pointA;
                *rowC = *rowA;

                *distA = candidateDist;
                *pointA = candidatePoint;
                *rowA = r;
            }
            else if (candidateDist < *distC)
            {
                *distC = candidateDist;
                *pointC = candidatePoint;
                *rowC = r;
            }
        }
    }
    if (*rowA == -1 || *rowC == -1)
        return 0;
    if (*rowA == *rowC)
        return 0;

    return 1;
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
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
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
    int edge_feature[MAX_ROWS][MAX_COLS] = {0}; // 特征矩阵
    int plane_feature[MAX_ROWS][MAX_COLS] = {0};
    extract_feature(lidarPointCloud, edge_feature, plane_feature);

    #ifdef FEATURE_DEBUG_PRINT
    FILE *csvFile = fopen("feature_data.csv", "w");
    fprintf(csvFile, "frame,row,col,x,y,z,is_edge,is_planar\n");
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            fprintf(csvFile, "%d,%d,%d,%lf,%lf,%lf,%d,%d\n",
                    lidarPointCloud->ToF_timestamps,
                    row, col,
                    lidarPointCloud->ToF_position[row][col].x,
                    lidarPointCloud->ToF_position[row][col].y,
                    lidarPointCloud->ToF_position[row][col].z,
                    edge_feature[row][col],
                    plane_feature[row][col]
                );
        }
    }
    fclose(csvFile);
    #endif

    // 初始化第0帧kdtree
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        // edge特征树
        Point edgePoints[MAX_COLS];
        size_t numEdgePoints;
        flattenPoints(attr->globalPointCloud[attr->frameCount].ToF_position[row], edge_feature[row], edgePoints, &numEdgePoints);
        attr->kdtree_edge_lastframe[row] = buildKDTree(edgePoints, numEdgePoints, 0); // 对每一行数据单独构建KD-Tree
        // plane特征树
        Point planePoints[MAX_COLS];
        size_t numPlanePoints;
        flattenPoints(attr->globalPointCloud[attr->frameCount].ToF_position[row], plane_feature[row], planePoints, &numPlanePoints);
        attr->kdtree_plane_lastframe[row] = buildKDTree(planePoints, numPlanePoints, 0);
    }

    attr->frameCount++;
}

// 配准定位
Pos slam_localization(SLAM_attr *attr, PointCloud *lidarPointCloud, Pos pos_predict, Pos pos_last)
{
    (void)pos_last; // 当前这一步里，优化基准统一围绕 pos_predict 展开

    double R[3][3];
    getRotationMatrix(DEG2RAD(pos_predict.roll), DEG2RAD(pos_predict.pitch), DEG2RAD(pos_predict.yaw), R);

    // 1. 提取当前帧特征
    int edge_feature[MAX_ROWS][MAX_COLS] = {0};
    int plane_feature[MAX_ROWS][MAX_COLS] = {0};
    extract_feature(lidarPointCloud, edge_feature, plane_feature);

    // 2. 先把当前帧点云按“基准位姿 pos_predict”变到全局坐标系
    // 后续优化的是围绕这个基准位姿的 6 维增量 delta
    double delta[6] = {0.0}; // [tx, ty, tz, rx, ry, rz]，其中旋转单位为 rad

    #ifdef FEATURE_DEBUG_PRINT
    FILE *csvFile = fopen("feature_data.csv", "a");
    for (int row = 0; row < MAX_ROWS; ++row) {
        for (int col = 0; col < MAX_COLS; ++col) {
            fprintf(csvFile, "%d,%d,%d,%lf,%lf,%lf,%d,%d\n",
                    lidarPointCloud->ToF_timestamps,
                    row, col,
                    lidarPointCloud->ToF_position[row][col].x,
                    lidarPointCloud->ToF_position[row][col].y,
                    lidarPointCloud->ToF_position[row][col].z,
                    edge_feature[row][col],
                    plane_feature[row][col]
                );
        }
    }
    fclose(csvFile);
    #endif
    
    // 转换当前帧点云到全局坐标系
    PointCloud transformed_pointcloud;
    for (int row = 0; row < MAX_ROWS; row++)
    {
        for (int col = 0; col < MAX_COLS; col++)
        {
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
    mapCoordinatesToLastFrame(&transformed_pointcloud, &pos_predict, delta, &positionInLastFrame);

    // 3. 配准
    // 外部迭代循环，设置20次迭代
    NeighborResult result[100] = {0};
    int CPcount = 0;

    const int max_iterations = 30;
    const int refind_interval = 1; // 这里直接每次迭代都重新找对应，保证 6DoF 下几何一致性
    const double lambda = 5e-3;    // 阻尼项，近似 LM

    double previousTotalError = INFINITY;

    // 鲁棒核与线/面平衡
    const double line_weight = 1.0;
    const double plane_weight = 1.5;
    const double huber_line_delta = 120.0; // mm
    const double huber_plane_delta = 60.0; // mm

    // 位姿步长保护，防止一次迭代走太猛
    const double max_step_t = 250.0;               // mm
    const double max_step_r = 10.0 * M_PI / 180.0; // rad

    double totalError = 0.0;
    int validEquationCount = 0;

    // 试着加了配准限制
    // edge
    double min_line_length = 50.0;           // 最小线特征长度，单位：mm
    double max_first_neighbor_dist = 800.0;  // 最近邻点距离阈值，单位：mm
    double max_second_neighbor_dist = 800.0; // 第二近邻点距离阈值，单位：mm
    int min_corr_count = 4;                  // 最小有效对应点数量，过少则跳过当前迭代
    // plane
    double max_plane_neighbor_dist = 1000.0; // 面特征点距离阈值，单位：mm
    double min_plane_area = 800.0;           // 三点张成平面的最小面积（叉积模长），单位：mm^2
    int plane_row_search_radius = 2;         // 搜索相邻行的范围，单位：行数

    for (int iter = 0; iter < 200; ++iter)
    {
        // 3.1 找最近点
        int refind_interval = 5;         // 每 n 次迭代重新搜索最近点
        if (iter % refind_interval == 0) // 注意：这里应该是iter == 0，或者每N次迭代重新搜索最近点
        {
            CPcount = 0; // 每次重新计算最近点时重置计数
            int flag = 0;

            mapCoordinatesToLastFrame(&transformed_pointcloud, &pos_predict, delta, &positionInLastFrame);

            for (int row = 0; row < MAX_ROWS; ++row)
            {
                for (int col = 0; col < MAX_COLS; ++col)
                {
                    if (edge_feature[row][col] == 1)
                    { // 只处理特征点
                        // 最近邻查找
                        Point currentPoint = positionInLastFrame.ToF_position[row][col];
                        Point lineA;
                        double bestDistA = INFINITY; // 初始设为一个非常大的值
                        nearestNeighborSearch(attr->kdtree_edge_lastframe[row], &currentPoint, &lineA, &bestDistA, 0);

                        if (bestDistA > max_first_neighbor_dist) // 如果最近邻点距离超过阈值，则跳过这个点
                            continue;

                        // 第二近邻查找（相邻行里查找）
                        Point candUp, candDown, lineB;
                        double distUp = INFINITY;
                        double distDown = INFINITY;
                        double bestDistB = INFINITY;

                        if (row > 0)
                            nearestNeighborSearch(attr->kdtree_edge_lastframe[row - 1], &lineA, &candUp, &distUp, 0);

                        if (row < MAX_ROWS - 1)
                            nearestNeighborSearch(attr->kdtree_edge_lastframe[row + 1], &lineA, &candDown, &distDown, 0);

                        if (distUp < distDown)
                        {
                            lineB = candUp;
                            bestDistB = distUp;
                        }
                        else
                        {
                            lineB = candDown;
                            bestDistB = distDown;
                        }
                        if (bestDistB > max_second_neighbor_dist) // 如果第二近邻点距离超过阈值，则跳过这个点
                            continue;

                        double line_dx = lineA.x - lineB.x;
                        double line_dy = lineA.y - lineB.y;
                        double line_dz = lineA.z - lineB.z;
                        double line_len = sqrt(line_dx * line_dx + line_dy * line_dy + line_dz * line_dz);

                        if (line_len < min_line_length)
                            continue;

                        // 放入最近邻数组
                        if (flag == CPcount)
                        {
                            result[CPcount].oriPoint = transformed_pointcloud.ToF_position[row][col];
                            result[CPcount].nearestPoint = lineA;
                            result[CPcount].distance = bestDistA;
                            result[CPcount].linePointA = lineA;
                            result[CPcount].linePointB = lineB;
                            result[CPcount].useLine = 1;
                            result[CPcount].usePlane = 0;
                            CPcount++;
                            continue;
                        }

                        // 每一行的其他点
                        int found = 0;
                        // 判断当前线特征是否已经在之前的点中被使用过；如果是，则更新对应的距离和最近点
                        for (int i = flag; i < CPcount; i++)
                        {
                            int sameAB = (result[i].linePointA.x == lineA.x) && (result[i].linePointA.y == lineA.y) && (result[i].linePointA.z == lineA.z) &&
                                         (result[i].linePointB.x == lineB.x) && (result[i].linePointB.y == lineB.y) && (result[i].linePointB.z == lineB.z);
                            int sameBA = (result[i].linePointA.x == lineB.x) && (result[i].linePointA.y == lineB.y) && (result[i].linePointA.z == lineB.z) &&
                                         (result[i].linePointB.x == lineA.x) && (result[i].linePointB.y == lineA.y) && (result[i].linePointB.z == lineA.z);
                            if (sameAB || sameBA)
                            {
                                if (result[i].distance > bestDistA)
                                {
                                    result[i].oriPoint = transformed_pointcloud.ToF_position[row][col];
                                    result[i].nearestPoint = lineA;
                                    result[i].distance = bestDistA;
                                    result[i].linePointA = lineA;
                                    result[i].linePointB = lineB;
                                    result[i].useLine = 1;
                                    result[i].usePlane = 0;
                                }
                                found = 1;
                                break;
                            }
                        }

                        if (!found)
                        {

                            result[CPcount].oriPoint = transformed_pointcloud.ToF_position[row][col];
                            result[CPcount].nearestPoint = lineA;
                            result[CPcount].distance = bestDistA;
                            result[CPcount].linePointA = lineA;
                            result[CPcount].linePointB = lineB;
                            result[CPcount].useLine = 1;
                            result[CPcount].usePlane = 0;
                            CPcount++;
                        }
                    }
                    if (plane_feature[row][col] == 1)
                    {
                        if (attr->kdtree_plane_lastframe[row] == NULL)
                            continue;

                        Point currentPoint = positionInLastFrame.ToF_position[row][col];
                        Point planePointA, planePointB, planePointC;
                        double distA = INFINITY, distB = INFINITY, distC = INFINITY;

                        // 当前行
                        nearestNeighborSearch(attr->kdtree_plane_lastframe[row], &currentPoint, &planePointB, &distB, 0);
                        if (distB > max_plane_neighbor_dist)
                            continue;

                        int planeRowA, planeRowC;
                        int foundPlaneNeighbors = findTwoNearestPlaneNeighbors(attr, &planePointB, row, plane_row_search_radius,
                                                                               &planeRowA, &planePointA, &distA, &planeRowC, &planePointC, &distC);
                        if (!foundPlaneNeighbors)
                            continue;
                        if (distA > max_plane_neighbor_dist || distC > max_plane_neighbor_dist)
                            continue;

                        // 退化检查：三点不能近共线
                        double abx = planePointB.x - planePointA.x;
                        double aby = planePointB.y - planePointA.y;
                        double abz = planePointB.z - planePointA.z;
                        double acx = planePointC.x - planePointA.x;
                        double acy = planePointC.y - planePointA.y;
                        double acz = planePointC.z - planePointA.z;
                        double nx = aby * acz - abz * acy;
                        double ny = abz * acx - abx * acz;
                        double nz = abx * acy - aby * acx;
                        double plane_area = sqrt(nx * nx + ny * ny + nz * nz);

                        if (plane_area < min_plane_area)
                            continue;

                        double planeScore = (distA + distB + distC) / 3.0;

                        // 去重：按三点组成的 plane patch 去重
                        int found = 0;
                        for (int i = 0; i < CPcount; i++)
                        {
                            if (!result[i].usePlane)
                                continue;
                            int samePlane = (result[i].planePointA.x == planePointA.x) && (result[i].planePointA.y == planePointA.y) && (result[i].planePointA.z == planePointA.z) && (result[i].planePointB.x == planePointB.x) && (result[i].planePointB.y == planePointB.y) && (result[i].planePointB.z == planePointB.z) && (result[i].planePointC.x == planePointC.x) && (result[i].planePointC.y == planePointC.y) && (result[i].planePointC.z == planePointC.z);
                            if (samePlane)
                            {
                                if (result[i].distance > planeScore)
                                {
                                    result[i].oriPoint = transformed_pointcloud.ToF_position[row][col];
                                    result[i].nearestPoint = planePointB;
                                    result[i].distance = planeScore;
                                    result[i].planePointA = planePointA;
                                    result[i].planePointB = planePointB;
                                    result[i].planePointC = planePointC;
                                    result[i].useLine = 0;
                                    result[i].usePlane = 1;
                                }
                                found = 1;
                                break;
                            }
                        }

                        if (!found && CPcount < MAX_ROWS * MAX_COLS)
                        {
                            result[CPcount].oriPoint = transformed_pointcloud.ToF_position[row][col];
                            result[CPcount].nearestPoint = planePointB;
                            result[CPcount].distance = planeScore;

                            result[CPcount].planePointA = planePointA;
                            result[CPcount].planePointB = planePointB;
                            result[CPcount].planePointC = planePointC;

                            result[CPcount].useLine = 0;
                            result[CPcount].usePlane = 1;
                            CPcount++;
                        }
                    }
                }
                flag = CPcount;
            }
            if (CPcount < min_corr_count)
            {
                printf("Too few correspondences: %d\n", CPcount);
                continue; // 配准的点太少，跳过当前迭代，继续下一次迭代
            }
            int edgeCorrCount = 0, planeCorrCount = 0;
            for (int i = 0; i < CPcount; i++)
            {
                if (result[i].useLine)
                    edgeCorrCount++;
                if (result[i].usePlane)
                    planeCorrCount++;
            }
            printf("Found %d correspondences (edge=%d, plane=%d)\n",
                   CPcount, edgeCorrCount, planeCorrCount);
#ifdef DEBUG_PRINT
            printf("时间戳: %zu\n", lidarPointCloud->ToF_timestamps);
            for (int i = 0; i < CPcount; i++)
            {
                printf("Result %d:\n", i);
                printf("  Original Point: (%.3f, %.3f, %.3f)\n", result[i].oriPoint.x, result[i].oriPoint.y, result[i].oriPoint.z);
                printf("  project Point: (%.3f, %.3f, %.3f)\n", result[i].oriPoint.x - transform[0], result[i].oriPoint.y - transform[1], result[i].oriPoint.z - transform[2]);
                printf("  Nearest Point: (%.3f, %.3f, %.3f)\n", result[i].nearestPoint.x, result[i].nearestPoint.y, result[i].nearestPoint.z);
                printf("ErrDistance = %.3f mm\n", result[i].distance);
            }
#endif
        }
        // 3.2 计算误差 d
        double ErrDistance[100];

        double H[6][6] = {0.0};
        double g[6] = {0.0};
        double totalError = 0.0;
        int validEquationCount = 0;

        for (int i = 0; i < CPcount; i++)
        {
            if (!result[i].useLine && !result[i].usePlane)
                continue;
            // 注意：这里计算的是映射后的点与最近点之间的距离
            Point p;
            applyIncrementalPoseToPoint(&result[i].oriPoint, &pos_predict, delta, &p);

            // 当前估计下，点相对“当前估计平移原点”的杠杆臂
            double sx = p.x - (pos_predict.x + delta[0]);
            double sy = p.y - (pos_predict.y + delta[1]);
            double sz = p.z - (pos_predict.z + delta[2]);

            if (result[i].useLine)
            {
                Point a = result[i].linePointA;
                Point b = result[i].linePointB;

                double vx = b.x - a.x;
                double vy = b.y - a.y;
                double vz = b.z - a.z;
                double vnorm = sqrt(vx * vx + vy * vy + vz * vz);
                if (vnorm < 1e-6)
                    continue;

                double ux = vx / vnorm;
                double uy = vy / vnorm;
                double uz = vz / vnorm;

                double mx = p.x - a.x;
                double my = p.y - a.y;
                double mz = p.z - a.z;

                double udotm = ux * mx + uy * my + uz * mz;

                // e = (I - uu^T)(p - a)
                double ex = mx - ux * udotm;
                double ey = my - uy * udotm;
                double ez = mz - uz * udotm;

                totalError += ex * ex + ey * ey + ez * ez;

                // A = I - uu^T
                double A[3][3];
                A[0][0] = 1.0 - ux * ux;
                A[0][1] = -ux * uy;
                A[0][2] = -ux * uz;
                A[1][0] = -uy * ux;
                A[1][1] = 1.0 - uy * uy;
                A[1][2] = -uy * uz;
                A[2][0] = -uz * ux;
                A[2][1] = -uz * uy;
                A[2][2] = 1.0 - uz * uz;

                // -[s]_x
                double Sneg[3][3];
                Sneg[0][0] = 0.0;
                Sneg[0][1] = sz;
                Sneg[0][2] = -sy;
                Sneg[1][0] = -sz;
                Sneg[1][1] = 0.0;
                Sneg[1][2] = sx;
                Sneg[2][0] = sy;
                Sneg[2][1] = -sx;
                Sneg[2][2] = 0.0;

                double J[3][6] = {0.0};

                // 平移块：A
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        J[r][c] = A[r][c];

                // 旋转块：A * (-[s]_x)
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        J[r][c + 3] = A[r][0] * Sneg[0][c] +
                                      A[r][1] * Sneg[1][c] +
                                      A[r][2] * Sneg[2][c];
                    }
                }

                double residual[3] = {ex, ey, ez};
                double r2 = ex * ex + ey * ey + ez * ez;
                double w = line_weight * huberWeightFromSquared(r2, huber_line_delta);

                totalError += w * r2;

                for (int r = 0; r < 3; ++r)
                {
                    validEquationCount++;
                    for (int m = 0; m < 6; ++m)
                    {
                        g[m] += w * J[r][m] * residual[r];
                        for (int n = 0; n < 6; ++n)
                            H[m][n] += w * J[r][m] * J[r][n];
                    }
                }
            }
            else if (result[i].usePlane)
            {
                Point a = result[i].planePointA;
                Point b = result[i].planePointB;
                Point c = result[i].planePointC;

                double abx = b.x - a.x;
                double aby = b.y - a.y;
                double abz = b.z - a.z;
                double acx = c.x - a.x;
                double acy = c.y - a.y;
                double acz = c.z - a.z;

                double nx = aby * acz - abz * acy;
                double ny = abz * acx - abx * acz;
                double nz = abx * acy - aby * acx;

                double norm_n = sqrt(nx * nx + ny * ny + nz * nz);
                if (norm_n < 1e-6)
                {
                    continue;
                }
                nx /= norm_n;
                ny /= norm_n;
                nz /= norm_n;

                double residual = (p.x - a.x) * nx + (p.y - a.y) * ny + (p.z - a.z) * nz;
                double r2 = residual * residual;
                double w = plane_weight * huberWeightFromSquared(r2, huber_plane_delta);

                totalError += w * r2;

                // J = [n^T, (s x n)^T]
                double j[6];
                j[0] = nx;
                j[1] = ny;
                j[2] = nz;
                j[3] = sy * nz - sz * ny;
                j[4] = sz * nx - sx * nz;
                j[5] = sx * ny - sy * nx;

                validEquationCount++;
                for (int m = 0; m < 6; ++m)
                {
                    g[m] += w * j[m] * residual;
                    for (int n = 0; n < 6; ++n)
                        H[m][n] += w * j[m] * j[n];
                }
            }
            else
            {
                ErrDistance[i] = 0.0;
            }
        }
        if (validEquationCount < min_corr_count)
        {
            printf("Too few valid equations: %d\n", validEquationCount);
            break;
        }

#ifdef DEBUG_PRINT
/*
        for (int i = 0; i < CPcount; i++)
        {
            printf("ErrDistance[%d] = %.3f m\n", i, ErrDistance[i]);
        }
*/
#endif

        // 阻尼 GN / LM
        double H_lm[6][6];
        double rhs[6];
        for (int i = 0; i < 6; ++i)
        {
            rhs[i] = -g[i];
            for (int j = 0; j < 6; ++j)
                H_lm[i][j] = H[i][j];

            H_lm[i][i] += lambda * (H[i][i] + 1e-6);
        }

        double step[6] = {0.0};
        if (!solveLinearSystem6x6(H_lm, rhs, step))
        {
            printf("6x6 solve failed, stop iteration.\n");
            break;
        }

        double rawStepTransNorm = sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]);
        double rawStepRotNorm = sqrt(step[3] * step[3] + step[4] * step[4] + step[5] * step[5]);

        clampPoseStep(step, max_step_t, max_step_r);

        for (int i = 0; i < 6; ++i)
            delta[i] += step[i];

        double stepTransNorm = sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]);
        double stepRotNorm = sqrt(step[3] * step[3] + step[4] * step[4] + step[5] * step[5]);

        int clamped = (fabs(rawStepTransNorm - stepTransNorm) > 1e-9) ||
                      (fabs(rawStepRotNorm - stepRotNorm) > 1e-9);

        printf("Iteration %d, Total Error: %.6f, step_t=%.6f mm, step_r=%.6f deg%s\n",
               iter,
               totalError,
               stepTransNorm,
               stepRotNorm * 180.0 / M_PI,
               clamped ? " [clamped]" : "");

        if (iter > 0 && fabs(previousTotalError - totalError) < 1e-3)
        {
            printf("收敛，停止迭代！\n");
            break;
        }

        if (stepTransNorm < 0.1 && stepRotNorm < 1e-4)
        {
            printf("位姿增量足够小，停止迭代！\n");
            break;
        }

        previousTotalError = totalError;
    }

    Pos pos_modified;
    pos_modified.x = pos_predict.x + delta[0];
    pos_modified.y = pos_predict.y + delta[1];
    pos_modified.z = pos_predict.z + delta[2];
    pos_modified.roll = pos_predict.roll + delta[3] * 180.0 / M_PI;
    pos_modified.pitch = pos_predict.pitch + delta[4] * 180.0 / M_PI;
    pos_modified.yaw = pos_predict.yaw + delta[5] * 180.0 / M_PI;

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
    for (int row = 0; row < MAX_ROWS; ++row)
    {
        for (int col = 0; col < MAX_COLS; ++col)
        {
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
    int edge_feature[MAX_ROWS][MAX_COLS] = {0};
    int plane_feature[MAX_ROWS][MAX_COLS] = {0};
    extract_feature(lidarPointCloud, edge_feature, plane_feature);

    for (int row = 0; row < MAX_ROWS; ++row)
    {
        Point edgePoints[MAX_COLS];
        size_t numEdgePoints;
        flattenPoints(attr->globalPointCloud[attr->frameCount].ToF_position[row], edge_feature[row], edgePoints, &numEdgePoints);
        attr->kdtree_edge_lastframe[row] = buildKDTree(edgePoints, numEdgePoints, 0); // 对每一行数据单独构建KD-Tree
        Point planePoints[MAX_COLS];
        size_t numPlanePoints;
        flattenPoints(attr->globalPointCloud[attr->frameCount].ToF_position[row], plane_feature[row], planePoints, &numPlanePoints);
        attr->kdtree_plane_lastframe[row] = buildKDTree(planePoints, numPlanePoints, 0);
        printf("row %d: edge=%zu, plane=%zu\n", row, numEdgePoints, numPlanePoints);
    }

    // 每次建完图，frameCount +1
    attr->frameCount++;
}