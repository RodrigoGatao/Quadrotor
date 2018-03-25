#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include"quad.h"

#include <QtCore/QObject>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPhongMaterial>

#include <Qt3DExtras/QPerVertexColorMaterial>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>

class scenemodifier: public QObject
{
    Q_OBJECT
private:
    double l;
    Qt3DCore::QEntity *m_lineEntity;
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DCore::QEntity *quad_arm1;
    Qt3DCore::QEntity *quad_arm2;
    Qt3DCore::QEntity *quad_motor1;
    Qt3DCore::QEntity *quad_motor2;
    Qt3DCore::QEntity *quad_motor3;
    Qt3DCore::QEntity *quad_motor4;
    Qt3DCore::QEntity *quad_up;

    matrixds des_state_s;
    matrixds old_state_s;
    matrixds old_des_state_s;
    matrixds state_s;

    MatrixXd a1;
    MatrixXd a2;
    MatrixXd a3;
    MatrixXd b1;
    MatrixXd b2;
    MatrixXd b3;

    bool dash_line = true;

public:
    explicit scenemodifier(Qt3DCore::QEntity *rootEntity);
    ~scenemodifier();

    /*****************
     *  Miscelanius Functions *
     ****************/
    /**
     * @brief create_grid plot a 3d grid
     */
    void create_grid();
    /**
     * @brief creat_sphere create a sphere in a desire position
     * @param x
     * @param y
     * @param z
     */
    void creat_sphere(double x,double y,double z);
    /**
     * @brief create_trajectories updates plot of the trajectory
     */
    void create_trajectories();
    /**
     * @brief update_plot update plot of the quadrotor
     */
    void update_plot();

    /*****************
     *  Set Functions *
     ****************/
    /**
     * @brief set_params defines the parameter of the quadrotor
     * @param quadparams
     */
    void set_params(params quadparams);
    /**
     * @brief set_state updates the state of the quadrotor
     * @param state
     * @param old_state
     * @param des_state
     * @param old_des_state
     */
    void set_state(matrixds state,matrixds old_state,matrixds des_state,matrixds old_des_state);


public slots:
    /**
     * @brief createLines creates and plot a new line
     * @param v0
     * @param v1
     * @param index
     * @param axis
     * @param lod_param
     */
    void createLines(const QVector3D &v0, const QVector3D &v1,
                         const unsigned int index, const bool axis, const QString &lod_param);


};

#endif // SCENEMODIFIER_H
