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

    /**
 * @brief The matrixds struct has the parameters of a matrix
 */
    struct matrixds{

    int c;
    int l;
    matrixd matrix;
};

    /**
     * @brief inv_transformation_matrix return the inverse matrix of a transformation matrix
     * @param roll
     * @param pitch
     * @param yaw
     * @return
     */
    matrixds inv_transformation_matrix(double roll, double pitch, double yaw);
    /**
     * @brief rotation_matrix returns the rotation matrix of the any Euler angles
     * @param roll
     * @param pitch
     * @param yaw
     * @return
     */
    matrixds rotation_matrix(double roll, double pitch, double yaw);
    /**
     * @brief transformation_matrix returns the transformation for any Euler angle
     * @param roll
     * @param pitch
     * @param yaw
     * @return
     */
    matrixds transformation_matrix(double roll, double pitch, double yaw);


    /**
     * @brief inverse_matrix return the inverse of any matrix
     * @param matrix
     * @return
     */
    matrixds inverse_matrix(matrixds matrix);
    /**
     * @brief multiple_matrix returns the multiplication of any scaler number or matrix
     * @param a
     * @param b
     * @return
     */
    matrixds multiple_matrix(double a, matrixds b);
    /**
     * @brief product_matrix returns the product of two matrixes
     * @param a
     * @param b
     * @return
     */
    matrixds product_matrix(matrixds a, matrixds b);
    /**
     * @brief sum_matrix returns the sum of two matrix
     * @param a
     * @param b
     * @return
     */
    matrixds sum_matrix(matrixds a, matrixds b);
    /**
     * @brief transposed_matrix returns the transposed of any matrix
     * @param a
     * @return
     */
    matrixds transposed_matrix(matrixds a);


    /**
     * @brief column_matrix returns a desired column of any matrix
     * @param m
     * @param a
     * @return
     */
    matrixds column_matrix(matrixds m, int a);
    /**
     * @brief line_matrix returns the desired line of any matrix
     * @param m
     * @param a
     * @return
     */
    matrixds line_matrix(matrixds m, int a);
    /**
     * @brief receive_matrix resize a matrix
     * @param l
     * @param c
     * @return
     */
    matrixds receive_matrix(int l, int c);


    /**
     * @brief mxd2mds returns the equivalent matrix in a format mds, with a format mxd
     * @param matrix
     * @return
     */
    matrixds mxd2mds(MatrixXd matrix);
    /**
     * @brief mds2mxd returns the equivalent matrix in a format mxd, with a format mds
     * @param matrix
     * @return
     */
    MatrixXd mds2mxd(matrixds matrix);


    /**
     * @brief print_Matrix print a matrix with cout
     * @param matrix
     */
    void print_Matrix(matrixds matrix);
    /**
     * @brief read_points read a .txt document
     * @param fname
     * @return
     */
    matrixds read_points(string fname);
    /**
     * @brief write_points write a text in a .txt file
     * @param fname
     * @param matrix
     */
    void write_points(string fname,matrixds matrix);

#endif // UTILS_H
