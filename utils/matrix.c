
#include "matrix.h"
#include <math.h>

// 矩阵乘法 C = A * B，维度为6x6
static void matrix_multiply(double A[6][6], double B[6][6], double C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 6; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 矩阵转置
static void matrix_transpose(double A[6][6], double AT[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            AT[i][j] = A[j][i];
        }
    }
}

// 矩阵加法 C = A + B
static void matrix_add(double A[6][6], double B[6][6], double C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

// 矩阵减法 C = A - B
static void matrix_subtract(double A[6][6], double B[6][6], double C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

// 矩阵求逆（6x6，使用高斯-约当消元法）
static int matrix_inverse(double A[6][6], double invA[6][6]) {
    double augmented[6][12];
    
    // 初始化增广矩阵 [A|I]
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            augmented[i][j] = A[i][j];
            augmented[i][j + 6] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // 高斯-约当消元
    for (int i = 0; i < 6; i++) {
        // 寻找主元
        double max_val = fabs(augmented[i][i]);
        int max_row = i;
        for (int k = i + 1; k < 6; k++) {
            if (fabs(augmented[k][i]) > max_val) {
                max_val = fabs(augmented[k][i]);
                max_row = k;
            }
        }
        
        // 如果主元接近0，矩阵奇异
        if (fabs(max_val) < 1e-12) {
            return -1; // 求逆失败
        }
        
        // 交换行
        if (max_row != i) {
            for (int j = 0; j < 12; j++) {
                double temp = augmented[i][j];
                augmented[i][j] = augmented[max_row][j];
                augmented[max_row][j] = temp;
            }
        }
        
        // 归一化当前行
        double pivot = augmented[i][i];
        for (int j = 0; j < 12; j++) {
            augmented[i][j] /= pivot;
        }
        
        // 消去其他行
        for (int k = 0; k < 6; k++) {
            if (k != i) {
                double factor = augmented[k][i];
                for (int j = 0; j < 12; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }
    
    // 提取逆矩阵
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            invA[i][j] = augmented[i][j + 6];
        }
    }
    
    return 0;
}
