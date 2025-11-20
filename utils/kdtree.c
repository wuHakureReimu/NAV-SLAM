
#include <stdio.h>
#include <stdlib.h>
#include <math.h>  
#include "kdtree.h"

// 获取分割轴（根据深度进行递归）
int getAxis(int depth) {
    return depth % 3;  // 0: x轴, 1: y轴, 2: z轴
}

// 计算欧几里得距离
double euclideanDistance(Point p1, Point p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

// nth_element实现
void nth_element(Point *points, size_t first, size_t last, size_t nth, int axis) {
    if (first >= last) return;
    
    size_t i = first;
    Point pivot = points[last];
    
    for (size_t j = first; j < last; j++) {
        double cmp = 0;
        switch (axis) {
            case 0: cmp = points[j].x - pivot.x; break;
            case 1: cmp = points[j].y - pivot.y; break;
            case 2: cmp = points[j].z - pivot.z; break;
        }
        if (cmp <= 0) {
            Point temp = points[i];
            points[i] = points[j];
            points[j] = temp;
            i++;
        }
    }
    
    Point temp = points[i];
    points[i] = points[last];
    points[last] = temp;
    
    if (i == nth) return;
    else if (i < nth) nth_element(points, i + 1, last, nth, axis);
    else nth_element(points, first, i - 1, nth, axis);
}

// 构建KD-Tree
KDNode* buildKDTree(Point *points, size_t numPoints, int depth) {
    if (numPoints == 0) return NULL;
    
    int axis = getAxis(depth);
    size_t median = numPoints / 2;
    
    nth_element(points, 0, numPoints - 1, median, axis);
    
    KDNode* node = (KDNode*)malloc(sizeof(KDNode));
    node->point = points[median];
    
    node->left = buildKDTree(points, median, depth + 1);
    node->right = buildKDTree(points + median + 1, numPoints - median - 1, depth + 1);
    
    return node;
}

void freeKDTree(KDNode* root) {
    if (!root) return;
    freeKDTree(root->left);
    freeKDTree(root->right);
    free(root);
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

// 最近邻搜索函数
void nearestNeighborSearch(KDNode* root, Point* target, Point* result, double* bestDist, int depth) {
    if (root == NULL) return;

    // 计算当前节点到目标点的距离
    double dist = euclideanDistance(root->point, *target);
    if (dist < *bestDist) {
        *bestDist = dist;
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
                           target->z - root->point.z)) < *bestDist) {
        nearestNeighborSearch(oppositeBranch, target, result, bestDist, depth + 1);
    }
}
