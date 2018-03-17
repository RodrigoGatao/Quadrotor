#include "utils.h"
#include"iostream"
#include"math.h"
#include"C:\Qt\Eigen\Eigen/Dense"



using namespace Eigen;
using namespace std;

void print_Matrix(matrixds m){
    int i = 0, j =0 ;
    for(i = 0; i < m.l; i++){
        for(j = 0;j < m.c; j++){
            cout << m.matrix[i][j] << " ";
        }
       cout <<endl;
    }
     cout << endl;
}

matrixds rotation_matrix(double roll, double pitch, double yaw)
{
    matrixds R;
    R.matrix = matrixd(3,vector<double>(3,0.0));
    R.matrix = {{(cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch)), ((-cos(roll)*sin(yaw))), (cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw))},
         {(cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch)), (cos(roll)*cos(yaw)), (sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw))},
         {(-cos(roll)*sin(pitch)), (sin(roll)), (cos(roll)*cos(pitch))}};
    R.l = 3;
    R.c = 3;

    return R;
}

matrixds transformation_matrix(double roll, double pitch, double yaw){
    matrixds T;
    T.matrix = matrixd(3,vector<double>(3,0.0));
    T.matrix = {{(cos(pitch)), (0), (-cos(roll)*sin(pitch))},
               {(0), (1), (sin(roll))},
               {(sin(pitch)), (0), (cos(roll)*cos(pitch))},
               };
    T.l = 3;
    T.c = 3;
 return T;
}

matrixds inv_transformation_matrix(double roll, double pitch, double yaw){
    matrixds inv;
    inv.matrix = matrixd(3,vector<double>(3,0.0));
    inv.matrix = {{(cos(pitch)), (0), (sin(pitch))},
                  {((sin(roll)*sin(pitch))/(cos(roll))), (1), ((-cos(pitch)*sin(roll))/(cos(roll)))},
                  {((-sin(pitch))/(cos(roll))), (0), ((cos(pitch))/cos(roll))},};
    inv.l = 3;
    inv.c = 3;
    return inv;
}

matrixds sum_matrix(matrixds a, matrixds b){
    int i = 0,j = 0;
    matrixds sum;
    sum.matrix = matrixd(a.l,vector<double>(a.c,0.0));
    for(i = 0; i < a.l; i++){
        for(j = 0; j < a.c; j++){
            sum.matrix[i][j] = a.matrix[i][j] + b.matrix[i][j];
        }
    }
    sum.l = a.l;
    sum.c = a.c;
    return sum;
}

matrixds transposed_matrix(matrixds a){
    int i = 0, j = 0;
    matrixds trans;
    trans.matrix = matrixd(a.c, vector<double>(a.l,0.0));
    for(i =0; i < a.l; i++){
        for(j = 0; j < a.c; j++){
            trans.matrix[j][i] = a.matrix[i][j];
        }
    }
    trans.l = a.c;
    trans.c = a.l;
    return trans;
}

matrixds product_matrix(matrixds a, matrixds b){
    int i = 0, j = 0, k = 0;
    double sum = 0;
    matrixds product;
    product.matrix = matrixd(a.l,vector<double>(b.c,0.0));
    if(a.c == b.l){
        for(i = 0; i < a.l; i++){
            for(j = 0; j < b.c; j++){
                for(k = 0;k < b.l; k++){
                    sum = sum + (a.matrix[i][k] * b.matrix[k][j]);
                }
                product.matrix[i][j] = sum;
                sum = 0;
            }
        }
    }
    product.l = a.l;
    product.c = b.c;
    return product;
}

matrixds multiple_matrix(double a, matrixds b){
    int i = 0, j = 0;
    matrixds multi;
    multi.matrix = matrixd(b.l,vector<double>(b.c,0.0));
    for(i = 0; i < b.l; i++){
        for(j = 0; j < b.c; j++){
            multi.matrix[i][j] = b.matrix[i][j] * a;
        }
    }
    multi.l = b.l;
    multi.c = b.c;
    return multi;
}

matrixds inverse_matrix(matrixds matrix){
    int i = 0, j = 0;
    matrixds inv;
    inv.matrix = matrixd(matrix.l,vector<double>(matrix.c,0.0));
    inv.l = matrix.l;
    inv.c = matrix.c;
    MatrixXd matrix2(matrix.l,matrix.c);
    if(matrix.l == matrix.c){
    for(i = 0; i < matrix.l; i++){
        for (j = 0; j < matrix.c; j++){
            matrix2(i,j) = matrix.matrix[i][j];
        }
    }
    FullPivLU<MatrixXd> lu(matrix2);
    MatrixXd lu2 = lu.inverse();
    for(i = 0; i < matrix.l; i++){
        for (j = 0; j < matrix.c; j++){
            inv.matrix[i][j] = lu2(i,j);
        }
     }
    }
    return inv;
}

matrixds line_matrix(matrixds m, int a){
    int i = 0;
    matrixds line;
    line.matrix = matrixd(1,vector<double>(m.c,0.0));
    line.l = 1;
    line.c = m.c;
    for(i = 0; i < m.c; i++){
        line.matrix[0][i] = m.matrix[a][i];
    }
    return line;
}

matrixds column_matrix(matrixds m, int a){
    int i = 0;
    matrixds column;
    column.matrix = matrixd(m.l,vector<double>(1,0.0));
    column.l = m.l;
    column.c = 1;
    for(i = 0; i < m.l; i++){
        column.matrix[i][0] = m.matrix[i][a];
    }
    return column;
}

matrixds receive_matrix(int l, int c){
    matrixds receive;
    receive.matrix = matrixd(l,vector<double>(c,0.0));
    receive.l = l;
    receive.c = c;
    return receive;
}

matrixds mxd2mds(MatrixXd matrix){
    int i = 0, j = 0;
    matrixds m = receive_matrix(matrix.rows(),matrix.cols());
    for(i = 0; i < matrix.rows(); i++){
        for(j = 0; j < matrix.cols(); j++){
            m.matrix[i][j] = matrix(i,j);
        }
    }
    return m;

}

MatrixXd mds2mxd(matrixds matrix){
    int i = 0, j = 0;
     MatrixXd m(matrix.l,matrix.c);
    for(i = 0; i < matrix.l; i++){
        for(j = 0; j < matrix.c; j++){
            m(i,j) = matrix.matrix[i][j];
        }
    }
    return m;

}

void write_points(string fname, matrixds matrix){
    ofstream output;
    output.open(fname);
    for(int i = 0; i < matrix.l; i++ ){
        for(int j = 0; j < matrix.c; j++){
            output << matrix.matrix[i][j] << "";
        }
    }
    output.close();
}

matrixds read_points(string fname){
    fstream input(fname,ios_base::in);
    if(!input){
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");
        msgBox.setText("The file could not be opened");
        msgBox.exec();
    }
    matrixds aux = receive_matrix(10,1);
    aux.l = 0;
    aux.c = 1;
    while(input >> aux.matrix[aux.l][0]){
        aux.l++;
    }

    return aux;
}
