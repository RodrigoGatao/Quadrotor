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

    controller *controlhandle;
    params quadparams;
    matrixds waypoints;

    MatrixXd gains_max;
    MatrixXd gains_min;

    double alpha;
    double beta;
    double ksi;
    double n;
    int t_max;

public:
    pso();

    /*****************
     *  Miscelanius Functions *
     ****************/
    /**
     * @brief fob returns the errors about each receive gains
     * @param gains
     * @return
     */
    double fob(matrixds gains);
    /**
     * @brief optimize the gains through the iteration of the fobs
     */
    void optimize();

    /*****************
     *  Set Functions *
     ****************/
    /**
     * @brief set_control defines the choose controller
     * @param a
     */
    void set_control(int a);
    /**
     * @brief set_params defines the parameters of the quadrotor
     * @param quadparams
     */
    void set_params(params quadparams);
    /**
     * @brief set_range_gains defines the maximum and minimum values to the gains
     */
    void set_range_gains();
    /**
     * @brief set_waypoints defines the the waypoints of a trajectory
     * @param waypoints
     */
    void set_waypoints(matrixds waypoints);
};

#endif // PSO_H
