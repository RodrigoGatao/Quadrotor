#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "quad.h"
#include"QMessageBox"
//#include"windows.h"
#include"pso.h"
#include<QVector>

#include <QMainWindow>
#include"scenemodifier.h"
#include<Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include<Qt3DInput/qinputaspect.h>
//#include<Qt3DInput/qInputAspect> para windows
#include<Qt3DCore/qentity.h>
#include<Qt3DRender/qcamera.h>
#include<Qt3DRender/qpointlight.h>
#include<Qt3DCore/qtransform.h>
#include<Qt3DExtras/qfirstpersoncameracontroller.h>

using namespace std;

namespace Ui {
class mainwindow;
}

class QCustomPlot;
class plot;

class mainwindow : public QMainWindow
{
    Q_OBJECT
private:
    Ui::mainwindow *ui;

    pso optimization;
    quad quadrotor;
    bool quad_isrunning = false;

    scenemodifier *modifier;
    Qt3DRender::QCamera *cameraEntity;
    Qt3DExtras::Qt3DWindow *view;


    plot *state_x, *state_y, *state_z, *state_roll, *state_pitch, *state_yaw;
    plot *des_state_x, *des_state_y, *des_state_z, *des_state_roll, *des_state_pitch, *des_state_yaw;
    QCustomPlot *m_plotx, *m_ploty, *m_plotz, *m_plotroll, *m_plotpitch, *m_plotyaw;


private slots:
    /**
     * @brief on_add_waypoints_clicked enables the add waypoints routine
     */
    void on_add_waypoints_clicked();
    /**
     * @brief on_change_controller_clicked enables the change controller routine
     */
    void on_change_controller_clicked();
    /**
     * @brief on_change_params_clicked enables the change params routine
     */
    void on_change_params_clicked();
    /**
     * @brief on_optimize_gain_clicked enables the PSO routine
     */
    void on_optimize_gain_clicked();
    /**
     * @brief on_reset_quad_clicked enables the reset quadrotor routine
     */
    void on_reset_quad_clicked();
    /**
     * @brief on_resetway_clicked enables the reset waypoints routine
     */
    void on_resetway_clicked();
    /**
     * @brief on_start_quad_clicked enables or disable the main routine
     */
    void on_start_quad_clicked();

public:
    explicit mainwindow(QWidget *parent = nullptr);
    ~mainwindow();
    /**
     * @brief clear_2dplot  clear all the graphics
     */
    void clear_2dplot();
    /**
     * @brief init_3dquad initializes the plot of the 3d graphics of quadrotor
     */
    void init_3dquad();
    /**
     * @brief init_2dplot initializes the 2d plot about x,y,z,roll,pitch,yaw
     */
    void init_2dplot();
    /**
     * @brief update_2dplot updates the 2d graphics about x,y,z,roll,pitch,yaw
     * @param state
     * @param des_state
     * @param t
     */
    void update_2dplot(matrixds state,matrixds des_state, double t);


public slots:
    /**
     * @brief update_quadStates update the plot of the quadrotor with the calculate states
     * @param state
     * @param old_state
     * @param des_state
     * @param old_des_state
     * @param t
     */
    void update_quadStates(matrixds state, matrixds old_state, matrixds des_state, matrixds old_des_state, double t);



};

#endif // MAINWINDOW_H
