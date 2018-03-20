#ifndef UTILS_H
#define UTILS_H
#include<eigen3/Eigen/Dense>
//#include"C:\Qt\Eigen3\Eigen3/Dense"
#include <utility>
#include<vector>
#define PI 3.14159265
#include"QMessageBox"
#include"fstream"
#include"iostream"

using namespace Eigen;
using namespace std;

typedef std::vector<std::vector<double>> matrixd;

struct matrixds{

    matrixd matrix;
    int l;
    int c;
};

    void write_points(string fname,matrixds matrix);
    void print_Matrix(matrixds matrix);
    matrixds read_points(string fname);
    matrixds rotation_matrix(double roll, double pitch, double yaw);
    matrixds transformation_matrix(double roll, double pitch, double yaw);
    matrixds inv_transformation_matrix(double roll, double pitch, double yaw);
    matrixds sum_matrix(matrixds a, matrixds b);
    matrixds transposed_matrix(matrixds a);
    matrixds product_matrix(matrixds a, matrixds b);
    matrixds multiple_matrix(double a, matrixds b);
    matrixds inverse_matrix(matrixds matrix);
    matrixds line_matrix(matrixds m, int a);
    matrixds column_matrix(matrixds m, int a);
    matrixds receive_matrix(int l, int c);
    matrixds mxd2mds(MatrixXd matrix);
    MatrixXd mds2mxd(matrixds matrix);

#endif // UTILS_H
