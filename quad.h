#ifndef QUAD_H
#define QUAD_H
#include "utils.h"
#include "controller.h"
#include<QThread>
using namespace std;

/**
 * @brief The params struct has the paramter of the quadrotor
 */
struct params{
    double b;
    double dt;
    double gravity;
    double k;
    double l;
    double mass;
    double Ixx;
    double Iyy;
    double Izz;
};


class quad: public QThread
{
    Q_OBJECT
private:
   matrixds angular_acc;
   matrixds angular_vel;//frame inicial (centro de massa), wx,wy,wz
   matrixds angular_vel_quad; //p,q,r
   matrixds linear_acc;
   matrixds linear_vel;
   matrixds orientation;
   matrixds position;
   matrixds R;

   matrixds b3;

   matrixds motor;
   params quadparams;
   matrixds waypoints;

   matrixds des_state;
   matrixds old_des_state;
   matrixds old_state;
   matrixds state;



   double t = 0;
   double iteration =  0;
   bool is_running = false;

protected:
    void run();

public:
    quad();
    ~quad();
    controller *controlhandle;

    /*****************
     *  Miscelanius Functions *
     ****************/
    /**
     * @brief init_params initialize the parameters of the quardrotor
     */
    void init_params();
    /**
     * @brief init_Quad initializes the state of quadrotor
     */
    void init_Quad();
    /**
     * @brief init_waypoints initialize the waypoints
     */
    void init_waypoints();
    /**
     * @brief model update the state of the quadrotor
     */
    void model();

    /*****************
     *  Get Functions *
     ****************/
    /**
     * @brief get_params returns the parameters about the quadrotor
     * @return
     */
    params get_params();
    /**
     * @brief get_waypoints returns the waypoints of the trajectory
     * @return
     */
    matrixds get_waypoints();

    /*****************
     *  Set Functions *
     ****************/
    /**
     * @brief set_controller defines the choose controller
     * @param a
     */
    void set_controller(int a);
    /**
     * @brief setParams defines the parameters of the quadrotor
     * @param select
     * @param value
     */
    void setParams(int select ,double value);
    /**
     * @brief set_run enables or disables the main routine
     * @param a
     */
    void set_run(bool a);
    /**
     * @brief set_waypoints defines the waypoints of the trajectory
     * @param matrix
     */
    void set_waypoints(matrixds matrix);

signals:/**
     * @brief emit_quadStates is a function to provides the connection between the objects of the quad and mainwindow classes
     * @param state
     * @param old_state
     * @param des_state
     * @param old_des_state
     * @param t
     */
    void emit_quadStates(matrixds state,matrixds old_state,matrixds des_state,matrixds old_des_state,double t);


};

#endif // QUAD_H
