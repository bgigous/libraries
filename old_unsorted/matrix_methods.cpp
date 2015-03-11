#pragma once

#include "matrix_methods.h"
#include <stdio.h>
using namespace std;

matrix2D m_subtract(matrix2D m1, matrix2D m2){
    int DIM1 = int(m1.size()), DIM2 = int(m1[0].size());  // Assumes multiplicable matrices
    for (int i=0; i<DIM1; i++){
        for (int j=0; j<DIM2; j++){
            m1[i][j] -= m2[i][j];
        }
    }
    return m1;  // Returns (m1-m2)
}

matrix1D m_subtract(matrix1D m1, matrix1D m2){
    int DIM = int(m1.size());  // Assumes multiplicable matrices
    for (int i=0; i<DIM; i++){
        m1[i] -= m2[i];
    }
    return m1;  // Returns (m1-m2)
}

matrix2D m_multiply(matrix2D m, double c){
    int DIM1 = int(m.size()), DIM2 = int(m[0].size());
    for (int i=0; i<DIM1; i++){
        for (int j=0; j<DIM2; j++){
            m[i][j] *= c;
        }
    }
    return m;   // Returns (m*c)
}

matrix1D m_multiply(matrix1D m, double c){
    int DIM = int(m.size());
    for (int i=0; i<DIM; i++){
        m[i] *= c;
    }
    return m;   // Returns (m*c)
}

matrix1D m_multiply(matrix2D m1, matrix1D m2){
    // m2 is a vertical matrix
    matrix1D m;

    for (int i=0; i<m1.size(); i++){
        double sum=0.0;
        for (int j=0; j<m1[0].size(); j++){
            sum+=m1[i][j]*m2[j];
        }
        m.push_back(sum);
    }
    return m;
}

matrix2D m_identity(int DIM){    // Assumes square matrix; takes only one dimension
    matrix2D m;
    for (int i=0; i<DIM; i++){
        matrix1D temp;
        for (int j=0; j<DIM; j++){
            if (i==j) temp.push_back(1.0);
            else temp.push_back(0.0);
        }
        m.push_back(temp);
    }
    return m;
}

matrix2D m_append(matrix2D leftmatrix, matrix2D rightmatrix){
    if (leftmatrix.size()!=rightmatrix.size()){
        printf("Size mismatch: returning first matrix.");
        return leftmatrix;
    }
    for (int i=0; i<leftmatrix.size(); i++){
        for (int j=0; j<rightmatrix[i].size(); j++){
            leftmatrix[i].push_back(rightmatrix[i][j]);
        }
    }
    return leftmatrix;
}

pair<matrix2D, matrix2D> m_separate(matrix2D m, int m_index){
    // Splits the matrix in two at m_index
    matrix2D leftmatrix, rightmatrix;
    for (int i=0; i<m.size(); i++){
        matrix1D left1D, right1D;
        for (int j=0; j<m[i].size(); j++){
            if (j<m_index) left1D.push_back(m[i][j]);
            else right1D.push_back(m[i][j]);
        }
        leftmatrix.push_back(left1D), rightmatrix.push_back(right1D);
    }
    return make_pair(leftmatrix, rightmatrix);
}

matrix2D m_rref(matrix2D m){
    int lead = 0;
    int rowcount = int(m.size());
    int columncount = int(m[0].size());
    for (int r=0; r<rowcount; r++){
        if (columncount<=lead) return m;
        int i=r;
        while (m[i][lead]==0){
            i++;
            if (rowcount==i){
                i=r;
                lead++;
                if (columncount==lead) return m;
            }
        }
        vector<double> swap = m[i];
        m[i] = m[r];
        m[r] = swap;
        if (m[r][lead]!=0) m[r] = m_multiply(m[r], (1.0/m[r][lead]));
        for (i=0; i<rowcount; i++){
            if (i!=r) m[i] = m_subtract(m[i], m_multiply(m[r], m[i][lead]));
        }
        lead++;
    }
    return m;
}

matrix2D m_inverse(matrix2D m){
    matrix2D I = m_identity(int(m.size()));
    m = m_append(m, I);
    m = m_rref(m);
    m = m_separate(m, (int(m[0].size())/2)).second; //TODO: MAKE SURE CARRIE KNOWS THAT THIS IS DOING INTEGER DIVISION.
    return m;
}

matrix2D m_convert_2D(double * array_first, int DIM){
    // Assumes square matrix/array
    matrix2D m;
    for (int i=0; i<DIM; i++){
        matrix1D mtemp;
        for (int j=0; j<DIM; j++){
            mtemp.push_back(array_first[i*DIM+j]);
        }
        m.push_back(mtemp);
    }
    return m;
}

matrix1D m_convert_1D(double * array_first, int DIM){
    // Assumes square matrix/array
    matrix1D m;
    for (int i=0; i<DIM; i++){
        m.push_back(array_first[i]);
    }
    return m;
}

vector<matrix2D> m_unf_matrix(int dim1, int dim2, int dim3, double val){
    vector<matrix2D> m3D;
    for (int i=0; i<dim1; i++){
        matrix2D m2D;
        for (int j=0; j<dim2; j++){
            matrix1D m1D;
            for (int k=0; k<dim3; k++){
                m1D.push_back(val);
            }
            m2D.push_back(m1D);
        }
        m3D.push_back(m2D);
    }
    return m3D;
}

matrix2D m_transpose(matrix2D m){
    matrix2D m_t;
    for (int i=0; i<m[0].size(); i++){
        matrix1D m_t_temp;
        for (int j=0; j<m.size(); j++){
            m_t_temp.push_back(m[j][i]);
        }
        m_t.push_back(m_t_temp);
    }
    return m_t;
}
