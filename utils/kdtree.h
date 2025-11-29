#ifndef KDTREE_H
#define KDTREE_H

#include <stddef.h>
#include "pointcloud.h"

typedef struct KDNode {
    Point point;
    struct KDNode* left;
    struct KDNode* right;
} KDNode;


typedef struct {
    Point oriPoint; //投影前的点
    Point nearestPoint;
    double distance;//投影后的当前帧点和最近邻点的距离
} NeighborResult;

// 构建kdtree
KDNode* buildKDTree(Point *points, size_t numPoints, int depth);

// 释放kdtree
void freeKDTree(KDNode* root);

// 最近邻搜索函数
void nearestNeighborSearch(KDNode* root, Point* target, Point* result, double* bestDist, int depth);

// 打印kdtree
void printKDTree(KDNode* root, int depth);

#endif