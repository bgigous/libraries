#pragma once
#include <vector>
#include <utility>

typedef std::vector<std::vector<double> > matrix2D;
typedef std::vector<double> matrix1D;

matrix2D m_subtract(matrix2D m1, matrix2D m2);
matrix1D m_subtract(matrix1D m1, matrix1D m2);
matrix2D m_multiply(matrix2D m, double c);
matrix1D m_multiply(matrix1D m, double c);
matrix1D m_multiply(matrix2D m1, matrix1D m2);
matrix2D m_identity(int DIM);
matrix2D m_append(matrix2D leftmatrix, matrix2D rightmatrix);
std::pair<matrix2D, matrix2D> m_separate(matrix2D m, int m_index);
matrix2D m_rref(matrix2D m);
matrix2D m_inverse(matrix2D m);
matrix1D m_convert_1D(double * array_first, int DIM);
matrix2D m_convert_2D(double * array_first, int DIM);
std::vector<matrix2D> m_unf_matrix(int dim1, int dim2, int dim3, double val);
matrix2D m_transpose(matrix2D);