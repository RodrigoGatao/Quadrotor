#ifndef QUAD_H
#define QUAD_H
#include "utils.h"
#include "controller.h"
#include<QThread>
using namespace std;

struct params{
    double mass;
    double l;
    double b;
    double k;
    double gravity;
    double dt;
    double Ixx;
    double Iyy;
    double Izz;
};

class quad: public QThread
{
    Q_OBJECT
private:
   matrixds position;
   matrixds orientation;
   matrixds linear_vel;
   matrixds linear_acc;
   matrixds angular_vel;//frame inicial (centro de massa), wx,wy,wz
   matrixds angular_vel_quad; //p,q,r
   matrixds angular_acc;
   matrixds state;
   matrixds old_state;
   matrixds des_state;
   matrixds old_des_state;
   params quadparams;
   matrixds R;
   matrixds motor;
   matrixds b3;
   matrixds waypoints;
   controller *controlhandle;
   double t = 0;
   double iteration =  0;
   bool is_running = false;

public:
    quad();
    void init_Quad();
    void init_waypoints();
    void init_params();
    void model();
    void setParams(int select ,double value);
    void set_controller(int a);
    void set_waypoints(matrixds matrix);
    matrixds get_waypoints();
    void set_run(bool a);
    params get_params();
    ~quad();

protected:
    void run();

signals:
    void emit_quadStates(matrixds state,matrixds old_state,matrixds des_state,matrixds old_des_state,double t);
};

#endif // QUAD_H
