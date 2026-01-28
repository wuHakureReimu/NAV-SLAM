
#ifndef SLAM_H
#define SLAM_H

#include <stddef.h>
#include "pointcloud.h"
#include "kdtree.h"

// SLAM算法维护的属性
typedef struct
{
    PointCloud globalPointCloud[100];   // 全局点云地图
    int frameCount;                     // 用这玩意和上面的数组形成一个丐版vector
    KDNode* kdtree_lastframe[MAX_ROWS];  // 存储上一帧点云的特征点KD-tree，每行一棵树，雷达坐标系
} SLAM_attr;


// 用第一帧数据初始化
void init_slam(SLAM_attr *attr, Pos pos, PointCloud *lidarPointCloud);

// 定位函数：根据输入的全局点云地图、当前帧雷达坐标系点云数据、当前帧预测位姿、上一帧位姿，配准得到输出位姿
Pos slam_localization(SLAM_attr *attr, PointCloud *lidarPointCloud, Pos pos_predict, Pos pos_last);

// 建图函数：根据输入的精确位姿和雷达点云数据，得到全局坐标系下的点云，将其保存到指向的点云地址
void slam_mapping(SLAM_attr *attr, Pos pos, PointCloud *lidarPointCloud);

#endif