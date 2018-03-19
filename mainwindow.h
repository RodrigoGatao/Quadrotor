#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "quad.h"
#include"QMessageBox"
#include"windows.h"
#include"pso.h"
#include<QVector>

#include <QMainWindow>
#include"scenemodifier.h"
#include<Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include<Qt3DInput/qInputAspect>
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

public:
    explicit mainwindow(QWidget *parent = nullptr);
    ~mainwindow();
    void init_3dquad();

private:
    Ui::mainwindow *ui;
    quad quadrotor;
    bool quad_isrunning = false;
    scenemodifier *modifier;
    Qt3DRender::QCamera *cameraEntity;
    Qt3DExtras::Qt3DWindow *view;
    pso optimization;


private slots:
    void on_start_quad_clicked();
    void on_reset_quad_clicked();
    void on_resetway_clicked();
    void on_change_params_clicked();
    void on_add_waypoints_clicked();
    void on_change_controller_clicked();
    void on_optimize_gain_clicked();
    QCustomPlot *mPlot;
    plot *posx;

public slots:
    void update_quadStates(matrixds state, matrixds old_state, matrixds des_state, matrixds old_des_state, double t);
};

#endif // MAINWINDOW_H
