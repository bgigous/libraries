#pragma once
#include <vector>
#include <iostream>
#include <chrono>
#include <random>
#include <string>
#include "../../Math/easymath.h"
#include "../../FileIO/FileIn.h"
#include "../../FileIO/FileOut.h"

class NeuralNet {
 public:
    NeuralNet() : evaluation(0.0), gamma_(0.9) {}
    ~NeuralNet() {}
    double evaluation;
    void mutate();  // different if child class

    void addInputs(int nToAdd);

    NeuralNet(int nInput, int nHidden, int nOutput, double gamma = 0.9);
    NeuralNet(std::vector<int> &, double gamma = 0.9);
    void train(matrix2d &O, matrix2d &T, double epsilon = 0.0,
        int iterations = 0);
    matrix1d predictBinary(matrix1d o);
    matrix1d predictContinuous(matrix1d o);
    matrix2d batchPredictBinary(matrix2d &O);
    matrix2d batchPredictContinuous(matrix2d &O);

 private:
    double gamma_;
    double mutStd;  // mutation standard deviation
    double mutationRate;  // probability that each connection is changed

    // container for all outputs on way through neural network: for FAST multiplication
    matrix2d matrix_multiplication_storage;
    std::vector<int> nodes_;  // number of nodes at each layer of the network
    matrix3d W;  // weights, W[interface][input][hidden(next unit)]. Without bias
    matrix3d Wbar;  // weights with bias;
    void setRandomWeights();  // sets weights randomly for the defined network
    void setMatrixMultiplicationStorage();  // sets storage for matrix multiplication. Must be called each time network structure is changed/initiated

    void  matrixMultiply(matrix1d &A, matrix2d &B, matrix1d &C);
    double SSE(matrix1d &myVector);
    int connections();
    double backProp(matrix1d &o, matrix1d &t);
    void feedForward(matrix1d &o, matrix2d &Ohat, matrix3d &D);

    matrix2d matrixMultiply(const matrix2d &A, const matrix2d &B);
    matrix2d matrixMultiply(const matrix1d&A, const matrix1d &B);
    matrix1d matrixMultiply(const matrix2d &A, const matrix1d &B);
    matrix1d matrixMultiply(const matrix1d &A, const matrix2d &B);
    void sigmoid(matrix1d *myVector);
    void cmp_int_fatal(int a, int b);



 protected:
    double randAddFanIn(double fan_in);
    double randSetFanIn(double fan_in);

 public:
    void save(std::string fileout);
    void load(std::string filein);
    void load(matrix1d node_info, matrix1d wt_info);
    void save(matrix1d *node_info, matrix1d *wt_info);
};
