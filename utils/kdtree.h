#ifndef KDTREE_H
#define KDTREE_H

#include <stddef.h>
#include "pointcloud.h"

typedef struct KDNode
{
    Point point;
    struct KDNode *left;
    struct KDNode *right;
} KDNode;

typedef struct
{
    Point oriPoint; // 投影前的点
    Point nearestPoint;
    double distance; // 投影后的当前帧点和最近邻点的距离

    Point linePointA;
    Point linePointB;
    int useLine; // 是否使用线特征

    Point planePointA;
    Point planePointB;
    Point planePointC;
    int usePlane; // 是否使用面特征
} NeighborResult;

// 构建kdtree
KDNode *buildKDTree(Point *points, size_t numPoints, int depth);

// 释放kdtree
void freeKDTree(KDNode *root);

// 最近邻搜索函数
void nearestNeighborSearch(KDNode *root, Point *target, Point *result, double *bestDist, int depth);

void nearestTwoNeighborSearch(KDNode *root, Point *target, Point *bestPoint1, double *bestDist1, Point *bestPoint2, double *bestDist2, int depth);
// 打印kdtree
void printKDTree(KDNode *root, int depth);

#endif