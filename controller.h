#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "utils.h"
#include "iostream"

using namespace std;

struct non_linear_gain{

    double kp_thrust;
    double kd_thrust;
    double kp_moment;
    double kd_moment;
};

struct linear_gain{

    double kp_xy;
    double kd_xy;
    double kp_z;
    double kd_z;
    double kd_thrust;
    double kp_moment;
    double kd_moment;
};

class controller
{
private:
    matrixds waypoints;
    matrixds motor;
    matrixds b3;
    matrixds I;
    non_linear_gain gt_gain;
    non_linear_gain tu_gain;
    linear_gain l_gain;
    double dt;
    double mass;
    double gravity;
    double k;
    double b;
    double l;
    int choose_controller = 3;

public:
    controller();
    void set_waypoints(matrixds m);
    matrixds trajhandle(double t);
    matrixds update_motor(double t, matrixds state);
    void geometric_tracking(double t, matrixds state);
    void linear_controller(double t, matrixds state);
    void thrust_up_controller(double t, matrixds state);
    matrixds next_state(double dt, matrixds state);
    void set_params(double mass1, double dt1, double gravity1, double Ixx, double Iyy, double Izz, double k1, double b1, double l1);
    void set_controller(int a);
    void set_gt_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);
    void set_tu_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);
    void set_l_gain(double kp_xy, double kd_xy,double kp_z, double kd_z, double kp_moment, double kd_moment);
};

#endif // CONTROLLER_H
