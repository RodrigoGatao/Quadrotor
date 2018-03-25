#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "utils.h"
#include "iostream"

using namespace std;

/**
 * @brief The non_linear_gain this struct has the gains to the geometrick tracking and the thrust up controller
 */
struct non_linear_gain{

    double kp_thrust;
    double kd_thrust;
    double kp_moment;
    double kd_moment;
};
/**
 * @brief The linear_gain struct this struct has the gains to the linear controller
 */
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
    matrixds b3;
    matrixds motor;
    matrixds waypoints;

    non_linear_gain gt_gain;
    linear_gain l_gain;
    non_linear_gain tu_gain;

    double b;
    double dt;
    double gravity;
    matrixds I;
    double k;
    double l;
    double mass;

    int choose_controller = 3;

    matrixds des_roll_pitch;

public:
    controller();

    /*****************
     *  Miscelanius Functions *
     *****************/
    /**

     * @brief geometric_tracking calculates the velocities about each motor on quadrotor, for the geometrick tracking controller
     * @param t
     * @param state
     */
    void geometric_tracking(double t, matrixds state);
    /**
     * @brief linear_controller calculates the velocities about each motor on quadrotor, for the linear controller
     * @param t
     * @param state
     */
    void linear_controller(double t, matrixds state);
    /**
     * @brief next_state return the next state of quadrotor, in the next instance time
     * @param dt
     * @param state
     * @return
     */
    matrixds next_state(double dt, matrixds state);
    /**
     * @brief thrust_up_controller calculates the velocities about each motor on quadrotor, for the thrust up controller
     * @param t
     * @param state
     */
    void thrust_up_controller(double t, matrixds state);
    /**
     * @brief trajhandle return the desire trajectory to the quadrotor
     * @param t
     * @return
     */
    matrixds trajhandle(double t);
    /**
     * @brief update_motor update the motor with the state in a current time
     * @param t
     * @param state
     * @return
     */
    matrixds update_motor(double t, matrixds state);

    /*****************
     *  Get Functions *
     ****************/
    /**
     * @brief get_des_ang return the new roll and pitch about each chose controller
     * @return
     */
    matrixds get_des_ang();

    /*****************
     *  Set Functions *
     ****************/
    /**
     * @brief set_controller defines what controller is running
     * @param a
     */
    void set_controller(int a);
    /**
     * @brief set_des_ang set the variables new roll and pitch, calculate by the controller
     * @param des_roll
     * @param des_pitch
     */
    void set_des_ang(double des_roll, double des_pitch);
    /**
     * @brief set_gt_gain defines the gains of the geometrick tracking controller
     * @param kp_thrust
     * @param kd_thrust
     * @param kp_moment
     * @param kd_moment
     */
    void set_gt_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);
    /**
     * @brief set_l_gain defines the gains of the linear controller
     * @param kp_xy
     * @param kd_xy
     * @param kp_z
     * @param kd_z
     * @param kp_moment
     * @param kd_moment
     */
    void set_l_gain(double kp_xy, double kd_xy,double kp_z, double kd_z, double kp_moment, double kd_moment);
    /**
     * @brief set_params define the params of the quadrotor
     * @param mass1
     * @param dt1
     * @param gravity1
     * @param Ixx
     * @param Iyy
     * @param Izz
     * @param k1
     * @param b1
     * @param l1
     */
    void set_params(double mass1, double dt1, double gravity1, double Ixx, double Iyy, double Izz, double k1, double b1, double l1);
    /**
     * @brief set_tu_gain defines the gains of the thrust up controller
     * @param kp_thrust
     * @param kd_thrust
     * @param kp_moment
     * @param kd_moment
     */
    void set_tu_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);
    /**
     * @brief set_waypoints defines the waypoints of trajectory
     * @param m
     */
    void set_waypoints(matrixds m);

};

#endif // CONTROLLER_H
