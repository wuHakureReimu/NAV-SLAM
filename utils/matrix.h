
#ifndef MATRIX_H
#define MATRIX_H

// 矩阵乘法 C = A * B，维度为6x6
static void matrix_multiply(double A[6][6], double B[6][6], double C[6][6]);

// 矩阵转置
static void matrix_transpose(double A[6][6], double AT[6][6]);

// 矩阵加法 C = A + B
static void matrix_add(double A[6][6], double B[6][6], double C[6][6]);

// 矩阵减法 C = A - B
static void matrix_subtract(double A[6][6], double B[6][6], double C[6][6]);

// 矩阵求逆（6x6，使用高斯-约当消元法）
static int matrix_inverse(double A[6][6], double invA[6][6]);

#endif