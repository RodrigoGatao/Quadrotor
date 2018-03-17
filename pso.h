#ifndef PSO_H
#define PSO_H
#include"utils.h"
#include"controller.h"
#include"quad.h"
#include"time.h"




class pso
{
private:
    int control = 3;
    matrixds waypoints;
    params quadparams;
    controller *controlhandle;
    MatrixXd gains_min;
    MatrixXd gains_max;
    double n;
    double alpha;
    double beta;
    double ksi;
    int t_max;
public:
    pso();
    void set_control(int a);
    double fob(matrixds gains);
    void set_waypoints(matrixds waypoints);
    void set_params(params quadparams);
    void optimize();
    void set_range_gains();
};

#endif // PSO_H
