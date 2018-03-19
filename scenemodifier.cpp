#include "scenemodifier.h"
#include <QtCore/QDebug>
#include<QMaterial>
#include"quad.h"
#include"iostream"



using namespace std;

scenemodifier::scenemodifier(Qt3DCore::QEntity *rootEntity)
    : m_rootEntity(rootEntity){

    state_s = receive_matrix(4,3);
    old_state_s = receive_matrix(4,3);
    des_state_s = receive_matrix(5,3);
    old_des_state_s = receive_matrix(5,3);

    a1.resize(3,1);
    a2.resize(3,1);
    a3.resize(3,1);
    a1 << 1, 0, 0;
    a2 << 0, 1, 0;
    a3 << 0, 0, 1;
    a1.resize(1,3);
    a2.resize(1,3);
    a3.resize(1,3);

    create_grid();
}

void scenemodifier::create_grid(){

    int x_mim = -3, x_max = 3, y_mim = -3, y_max = 3, i,z_mim = 0,z_max = 3;
    for(i = x_mim; i <= x_max; i++){

        this->createLines(QVector3D(i,y_mim,0),QVector3D(i,y_max,0),3,true,"");
    }
    for(i = y_mim; i <= y_max; i++){

        this->createLines(QVector3D(x_mim,i,0),QVector3D(x_max,i,0),3,true,"");
    }
    for(i = x_mim; i <= x_max; i++){

        this->createLines(QVector3D(i,y_mim,z_mim),QVector3D(i,y_mim,z_max),3,true,"");
    }
    for(i = y_mim; i <= y_max; i++){

        this->createLines(QVector3D(x_mim,i,z_mim),QVector3D(x_mim,i,z_max),3,true,"");
    }
    for(i = z_mim; i <= z_max; i++){

        this->createLines(QVector3D(x_mim,y_mim,i),QVector3D(x_mim,y_max,i),3,true,"");
    }
    for(i = z_mim; i <= z_max; i++){

        this->createLines(QVector3D(x_mim,y_mim,i),QVector3D(x_max,y_mim,i),3,true,"");
    }

}

void scenemodifier::set_state(matrixds state,matrixds old_state,matrixds des_state,matrixds old_des_state){
    state_s = state;
    old_state_s = old_state;
    des_state_s = des_state;
    old_des_state_s = old_des_state;

}

void scenemodifier::set_params(params quadparams){
    l = quadparams.l;

    MatrixXd pos(1,3);
    MatrixXd motor1_pos(1,2), motor2_pos(1,2), motor3_pos(1,2), motor4_pos(1,2), up_pos(1,3);

    double roll = state_s.matrix[2][0];
    double pitch = state_s.matrix[2][1];
    double yaw = state_s.matrix[2][2];

    pos << state_s.matrix[0][0], state_s.matrix[0][1], state_s.matrix[0][2];

    b1 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a1.transpose()).transpose();
    b2 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a2.transpose()).transpose();
    b3 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a3.transpose()).transpose();

    motor1_pos = pos + b1*l;
    motor2_pos = pos + b2*l;
    motor3_pos = pos - b1*l;
    motor4_pos = pos - b2*l;
    up_pos = pos + b3*l/4;

    //Motor1 shape data
    Qt3DExtras::QCylinderMesh *motor1 = new Qt3DExtras::QCylinderMesh();
    motor1->setRadius(l/5);
    motor1->setLength(l/20);
    motor1->setRings(100);
    motor1->setSlices(20);

    //Motor1 Transform
    Qt3DCore::QTransform *motor1Transform = new Qt3DCore::QTransform();
    motor1Transform->setScale(1.0f);
    motor1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor1Transform->setTranslation(QVector3D(motor1_pos(0,0),motor1_pos(0,1),motor1_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor1Material = new Qt3DExtras::QPhongMaterial();
    motor1Material->setDiffuse(QColor("Red"));
    quad_motor1 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor1->addComponent(motor1);
    quad_motor1->addComponent(motor1Material);
    quad_motor1->addComponent(motor1Transform);

    //Motor2 shape data
    Qt3DExtras::QCylinderMesh *motor2 = new Qt3DExtras::QCylinderMesh();
    motor2->setRadius(l/5);
    motor2->setLength(l/20);
    motor2->setRings(100);
    motor2->setSlices(20);

    //motor2 Transform
    Qt3DCore::QTransform *motor2Transform = new Qt3DCore::QTransform();
    motor2Transform->setScale(1.0f);
    motor2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor2Transform->setTranslation(QVector3D(motor2_pos(0,0),motor2_pos(0,1),motor2_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor2Material = new Qt3DExtras::QPhongMaterial();
    motor2Material->setDiffuse(QColor("black"));
    quad_motor2 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor2->addComponent(motor2);
    quad_motor2->addComponent(motor2Material);
    quad_motor2->addComponent(motor2Transform);

    //motor3 shape data
    Qt3DExtras::QCylinderMesh *motor3 = new Qt3DExtras::QCylinderMesh();
    motor3->setRadius(l/5);
    motor3->setLength(l/20);
    motor3->setRings(100);
    motor3->setSlices(20);

    //motor3 Transform
    Qt3DCore::QTransform *motor3Transform = new Qt3DCore::QTransform();
    motor3Transform->setScale(1.0f);
    motor3Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor3Transform->setTranslation(QVector3D(motor3_pos(0,0),motor3_pos(0,1),motor3_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor3Material = new Qt3DExtras::QPhongMaterial();
    motor3Material->setDiffuse(QColor("black"));
    quad_motor3 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor3->addComponent(motor3);
    quad_motor3->addComponent(motor3Material);
    quad_motor3->addComponent(motor3Transform);

    //motor4 shape data
    Qt3DExtras::QCylinderMesh *motor4 = new Qt3DExtras::QCylinderMesh();
    motor4->setRadius(l/5);
    motor4->setLength(l/20);
    motor4->setRings(100);
    motor4->setSlices(20);

    //motor4 Transform
    Qt3DCore::QTransform *motor4Transform = new Qt3DCore::QTransform();
    motor4Transform->setScale(1.0f);
    motor4Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor4Transform->setTranslation(QVector3D(motor4_pos(0,0),motor4_pos(0,1),motor4_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor4Material = new Qt3DExtras::QPhongMaterial();
    motor4Material->setDiffuse(QColor("black"));
    quad_motor4 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor4->addComponent(motor4);
    quad_motor4->addComponent(motor4Material);
    quad_motor4->addComponent(motor4Transform);


    //up shape data
    Qt3DExtras::QCylinderMesh *up = new Qt3DExtras::QCylinderMesh();
    up->setRadius(l/20);
    up->setLength(l/2);
    up->setRings(100);
    up->setSlices(20);

    //up Transform
    Qt3DCore::QTransform *upTransform = new Qt3DCore::QTransform();
    upTransform->setScale(1.0f);
    upTransform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    upTransform->setTranslation(QVector3D(up_pos(0,0),up_pos(0,1),up_pos(0,2)));

    Qt3DExtras::QPhongMaterial *upMaterial = new Qt3DExtras::QPhongMaterial();
    upMaterial->setDiffuse(QColor("black"));
    quad_up = new Qt3DCore::QEntity(m_rootEntity);
    quad_up->addComponent(up);
    quad_up->addComponent(upMaterial);
    quad_up->addComponent(upTransform);

    //arm1 shape data
    Qt3DExtras::QCylinderMesh *arm1 = new Qt3DExtras::QCylinderMesh();
    arm1->setRadius(l/20);
    arm1->setLength(l*2);
    arm1->setRings(100);
    arm1->setSlices(20);

    //arm1 Transform
    Qt3DCore::QTransform *arm1Transform = new Qt3DCore::QTransform();
    arm1Transform->setScale(1.0f);
    arm1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm1Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    Qt3DExtras::QPhongMaterial *arm1Material = new Qt3DExtras::QPhongMaterial();
    arm1Material->setDiffuse(QColor("black"));
    quad_arm1 = new Qt3DCore::QEntity(m_rootEntity);
    quad_arm1->addComponent(arm1);
    quad_arm1->addComponent(arm1Material);
    quad_arm1->addComponent(arm1Transform);

    //arm2 shape data
    Qt3DExtras::QCylinderMesh *arm2 = new Qt3DExtras::QCylinderMesh();
    arm2->setRadius(l/20);
    arm2->setLength(l*2);
    arm2->setRings(100);
    arm2->setSlices(20);

    //arm2 Transform
    Qt3DCore::QTransform *arm2Transform = new Qt3DCore::QTransform();
    arm2Transform->setScale(1.0f);
    arm2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(-b1(0,0),-b1(0,1),-b1(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm2Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    Qt3DExtras::QPhongMaterial *arm2Material = new Qt3DExtras::QPhongMaterial();
    arm2Material->setDiffuse(QColor("black"));
    quad_arm2 = new Qt3DCore::QEntity(m_rootEntity);
    quad_arm2->addComponent(arm2);
    quad_arm2->addComponent(arm2Material);
    quad_arm2->addComponent(arm2Transform);
}

void scenemodifier::create_trajectories(){

    this->createLines(QVector3D(old_state_s.matrix[0][0],old_state_s.matrix[0][1],old_state_s.matrix[0][2]),QVector3D(state_s.matrix[0][0],state_s.matrix[0][1],state_s.matrix[0][2]),1,true,"");
    if(dash_line){
         this->createLines(QVector3D(old_des_state_s.matrix[0][0],old_des_state_s.matrix[0][1],old_des_state_s.matrix[0][2]),QVector3D(des_state_s.matrix[0][0],des_state_s.matrix[0][1],des_state_s.matrix[0][2]),4,true,"");
         dash_line = false;
    }
    else{
        dash_line = true;
    }
}

scenemodifier::~scenemodifier()
{
}

void scenemodifier::createLines(const QVector3D &v0, const QVector3D &v1,
                                const unsigned int index, const bool axis, const QString &lod_param)
{
    Qt3DRender::QGeometryRenderer *line_mesh = new Qt3DRender::QGeometryRenderer();
    Qt3DRender::QGeometry *geometry = new Qt3DRender::QGeometry(line_mesh);

    Qt3DRender::QBuffer *vertexDataBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, geometry);
    Qt3DRender::QBuffer *indexDataBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::IndexBuffer, geometry);

    QByteArray vertexBufferData;
    vertexBufferData.resize(2 * (3 + 3) * sizeof(float));

    QVector<QVector3D> vertices = (!axis ? QVector<QVector3D>() << v0 << QVector3D (0.5f, 0.0f, 0.5f) << v1  << QVector3D (0.5f, 0.0f, 0.5f)
                                         : (index==0 ? QVector<QVector3D>() << v0 << QVector3D (1.0f, 0.0f, 0.0f) << v1  << QVector3D (1.0f, 0.0f, 0.0f)
                                                      : (index==1 ? QVector<QVector3D>() << v0 << QVector3D (0.0f, 0.0f, 0.0f) << v1  << QVector3D (0.0f, 0.0f, 0.0f)
                                                                  : (index==2 ? QVector<QVector3D>() << v0 << QVector3D (0.0f, 0.0f, 1.0f) << v1  << QVector3D (0.0f, 0.0f, 1.0f)
                                                                              : QVector<QVector3D>() << v0 << QVector3D (0.0f, 1.0f, 1.0f) << v1  << QVector3D (0.0f, 1.0f, 1.0f)))));

    float *rawVertexArray = reinterpret_cast<float *>(vertexBufferData.data());
    int idx = 0;

    Q_FOREACH (const QVector3D &v, vertices) {
        rawVertexArray[idx++] = v.x();
        rawVertexArray[idx++] = v.y();
        rawVertexArray[idx++] = v.z();
    }

    QByteArray indexBufferData;
    indexBufferData.resize(2 * 3 * sizeof(ushort));
    ushort *rawIndexArray = reinterpret_cast<ushort *>(indexBufferData.data());

    rawIndexArray[0] = 0;
    rawIndexArray[1] = 1;

    vertexDataBuffer->setData(vertexBufferData);
    indexDataBuffer->setData(indexBufferData);

    // Attributes
    Qt3DRender::QAttribute *positionAttribute = new Qt3DRender::QAttribute();
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(vertexDataBuffer);
    positionAttribute->setDataType(Qt3DRender::QAttribute::Float);
    positionAttribute->setDataSize(3);
    positionAttribute->setByteOffset(0);
    positionAttribute->setByteStride(6 * sizeof(float));
    positionAttribute->setCount(2);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());

    Qt3DRender::QAttribute *colorAttribute = new Qt3DRender::QAttribute();
    colorAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    colorAttribute->setBuffer(vertexDataBuffer);
    colorAttribute->setDataType(Qt3DRender::QAttribute::Float);
    colorAttribute->setDataSize(3);
    colorAttribute->setByteOffset(3 * sizeof(float));
    colorAttribute->setByteStride(6 * sizeof(float));
    colorAttribute->setCount(2);
    colorAttribute->setName(Qt3DRender::QAttribute::defaultColorAttributeName());

    Qt3DRender::QAttribute *indexAttribute = new Qt3DRender::QAttribute();
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexDataBuffer);
    indexAttribute->setDataType(Qt3DRender::QAttribute::UnsignedShort);
    indexAttribute->setDataSize(1);
    indexAttribute->setByteOffset(0);
    indexAttribute->setByteStride(0);
    indexAttribute->setCount(2);

    geometry->addAttribute(positionAttribute);
    geometry->addAttribute(colorAttribute);
    geometry->addAttribute(indexAttribute);

    line_mesh->setInstanceCount(1);
    line_mesh->setIndexOffset(0);
    line_mesh->setFirstInstance(0);
    line_mesh->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    line_mesh->setGeometry(geometry);
    line_mesh->setVertexCount(2);

    // Material
    Qt3DRender::QMaterial *material = new Qt3DExtras::QPerVertexColorMaterial(m_rootEntity);

    // Line Entity
    m_lineEntity = new Qt3DCore::QEntity(m_rootEntity);
    m_lineEntity->addComponent(line_mesh);
    m_lineEntity->addComponent(material);
    if(index == 3){
        Qt3DExtras::QPhongMaterial *lineMaterial = new Qt3DExtras::QPhongMaterial();
        lineMaterial->setAmbient(QColor("Gray"));
        m_lineEntity->addComponent(lineMaterial);
    }
    else if(index == 4){
        Qt3DExtras::QPhongMaterial *lineMaterial = new Qt3DExtras::QPhongMaterial();
        lineMaterial->setAmbient(QColor("Darkred"));
        m_lineEntity->addComponent(lineMaterial);
    }

    if(!axis)
    {
        m_lineEntity->setObjectName(QString::number(index)+ ":" +lod_param);
        Qt3DRender::QObjectPicker createObjectPickerForEntity(m_lineEntity);
    }
}

void scenemodifier::update_plot(){

    MatrixXd pos(1,3);
    MatrixXd motor1_pos(1,2), motor2_pos(1,2), motor3_pos(1,2), motor4_pos(1,2), up_pos(1,3);

    double roll = state_s.matrix[2][0];
    double pitch = state_s.matrix[2][1];
    double yaw = state_s.matrix[2][2];

    create_trajectories();

    pos << state_s.matrix[0][0], state_s.matrix[0][1], state_s.matrix[0][2];

    b1 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a1.transpose()).transpose();
    b2 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a2.transpose()).transpose();
    b3 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a3.transpose()).transpose();

    motor1_pos = pos + b1*l;
    motor2_pos = pos + b2*l;
    motor3_pos = pos - b1*l;
    motor4_pos = pos - b2*l;
    up_pos = pos + b3*l/4;

    //Motor1 Transform
    Qt3DCore::QTransform *motor1Transform = new Qt3DCore::QTransform();
    motor1Transform->setScale(1.0f);
    motor1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor1Transform->setTranslation(QVector3D(motor1_pos(0,0),motor1_pos(0,1),motor1_pos(0,2)));

    //motor2 Transform
    Qt3DCore::QTransform *motor2Transform = new Qt3DCore::QTransform();
    motor2Transform->setScale(1.0f);
    motor2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor2Transform->setTranslation(QVector3D(motor2_pos(0,0),motor2_pos(0,1),motor2_pos(0,2)));

    //motor3 Transform
    Qt3DCore::QTransform *motor3Transform = new Qt3DCore::QTransform();
    motor3Transform->setScale(1.0f);
    motor3Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor3Transform->setTranslation(QVector3D(motor3_pos(0,0),motor3_pos(0,1),motor3_pos(0,2)));

    //motor4 Transform
    Qt3DCore::QTransform *motor4Transform = new Qt3DCore::QTransform();
    motor4Transform->setScale(1.0f);
    motor4Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor4Transform->setTranslation(QVector3D(motor4_pos(0,0),motor4_pos(0,1),motor4_pos(0,2)));

    //up Transform
    Qt3DCore::QTransform *upTransform = new Qt3DCore::QTransform();
    upTransform->setScale(1.0f);
    upTransform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    upTransform->setTranslation(QVector3D(up_pos(0,0),up_pos(0,1),up_pos(0,2)));

    //arm1 Transform
    Qt3DCore::QTransform *arm1Transform = new Qt3DCore::QTransform();
    arm1Transform->setScale(1.0f);
    arm1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm1Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    //arm2 Transform
    Qt3DCore::QTransform *arm2Transform = new Qt3DCore::QTransform();
    arm2Transform->setScale(1.0f);
    arm2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(-b1(0,0),-b1(0,1),-b1(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm2Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    quad_motor1->addComponent(motor1Transform);
    quad_motor2->addComponent(motor2Transform);
    quad_motor3->addComponent(motor3Transform);
    quad_motor4->addComponent(motor4Transform);
    quad_up->addComponent(upTransform);
    quad_arm1->addComponent(arm1Transform);
    quad_arm2->addComponent(arm2Transform);

}

void scenemodifier::creat_sphere(double x, double y, double z){
    // Sphere shape data
        Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
        sphereMesh->setRings(20);
        sphereMesh->setSlices(20);
        sphereMesh->setRadius(0.015);

        // Sphere mesh transform
        Qt3DCore::QTransform *sphereTransform = new Qt3DCore::QTransform();

        sphereTransform->setScale(1.0f);
        sphereTransform->setTranslation(QVector3D(x,y,z));

        Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
        sphereMaterial->setDiffuse(QColor("darkred"));

        Qt3DCore::QEntity *m_sphereEntity;
        // Sphere
        m_sphereEntity = new Qt3DCore::QEntity(m_rootEntity);
        m_sphereEntity->addComponent(sphereMesh);
        m_sphereEntity->addComponent(sphereMaterial);
        m_sphereEntity->addComponent(sphereTransform);
}
