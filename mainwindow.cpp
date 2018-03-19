#include"mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"
#include "quad.h"
#include "utils.h"
#include"scenemodifier.h"
#include"qcustomplot.h"
#include"plot.h"


using namespace std;

mainwindow::mainwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mainwindow)
{
    qRegisterMetaType<matrixds>("matrixds");
    ui->setupUi(this);
    //mPlot = new QCustomPlot();
//    ui->layout_x->addWidget(mPlot);
//    ui->layout_x->setMargin(0);
//    mPlot->xAxis->setRange(0,3.0);
//    mPlot->xAxis->setRange(-3.0,3.0);
//    posx = new plot(mPlot);
    ui->params_options->addItem("Mass");
    ui->params_options->addItem("L");
    ui->params_options->addItem("B");
    ui->params_options->addItem("K");
    ui->params_options->addItem("Ixx");
    ui->params_options->addItem("Iyy");
    ui->params_options->addItem("Izz");
    ui->LcdTempo->setPalette(Qt::black);
    ui->controller_options->addItem("Linear");
    ui->controller_options->addItem("Thrust up");
    ui->controller_options->addItem("Geometric Tracking");
    view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor("White"));
    QWidget *container = QWidget::createWindowContainer(view);
    ui->vlayout->setAlignment(Qt::AlignJustify);
    ui->hlayout->addWidget(container,1);

    init_3dquad();
    connect(&quadrotor, SIGNAL(emit_quadStates(matrixds,matrixds,matrixds,matrixds,double)), this, SLOT(update_quadStates(matrixds,matrixds,matrixds,matrixds,double)), Qt::QueuedConnection);
}

mainwindow::~mainwindow()
{
    delete ui;
}

void mainwindow::update_quadStates(matrixds state,matrixds old_state,matrixds des_state,matrixds old_des_state,double t){
//    posx->set_x(t);
//    posx->set_y(des_state.matrix[0][0]);
//    posx->draw_graph();
//    if(t>3){
//        mPlot->xAxis->setRange(0,t);
//    }
//    mPlot->replot();
//    ui->layout_x->update();

    ui->LcdTempo->display(t);
    modifier->set_state(state,old_state,des_state,old_des_state);
    modifier->update_plot();

}

//Altera Botão start
void mainwindow::on_start_quad_clicked()
{
    if(quad_isrunning == false)
    {
        ui->start_quad->setText("Stop");
        quad_isrunning = true;
        quadrotor.set_run(1);
        //Inicia thread
        quadrotor.start();
    }else
    {
        ui->start_quad->setText("Play");
        quad_isrunning = false;
        //Finaliza thread alterando variavel booleana is_running
        quadrotor.set_run(0);
        cout << 0 << endl;
    }
}

//Pegando valores de parametros que podem ser alterados
void mainwindow::on_change_params_clicked(){

    string quad_params_decision = ui->params_options->currentText().toUtf8().constData();//Pega valor da comboBox
    string value_s = ui->Params_value->text().toUtf8().constData(); //Paga valor do parametro
    double value = atof(value_s.c_str());//StringToDouble

    int num_params;

    if(quad_params_decision == "Mass"){
        num_params = 1;
    }else if(quad_params_decision == "L"){
            num_params = 2;

    }else if(quad_params_decision == "B"){
            num_params = 3;

    }else if(quad_params_decision == "K"){
            num_params = 4;

    }else if(quad_params_decision == "Ixx"){
            num_params = 5;

    }else if(quad_params_decision == "Iyy"){
            num_params = 6;

    }else if(quad_params_decision == "Izz"){
            num_params = 7;
    }
    quadrotor.setParams(num_params,value);
    quadrotor.init_Quad();
    init_3dquad();
    ui->LcdTempo->display(0);
}

void mainwindow::on_add_waypoints_clicked(){

    string w_x = ui->x->text().toUtf8().constData(); //Paga valor do parametro
    string w_y = ui->y->text().toUtf8().constData(); //Paga valor do parametro
    string w_z = ui->z->text().toUtf8().constData(); //Paga valor do parametro
    string w_yaw = ui->yaw->text().toUtf8().constData(); //Paga valor do parametro
    string w_time = ui->time->text().toUtf8().constData(); //Paga valor do parametro
    double waypoint_x = atof(w_x.c_str());//StringToDouble
    double waypoint_y = atof(w_y.c_str());//StringToDouble
    double waypoint_z = atof(w_z.c_str());//StringToDouble
    double waypoint_yaw = atof(w_yaw.c_str());//StringToDouble
    double waypoint_time = atof(w_time.c_str());//StringToDouble
    matrixds waypoints;
    waypoints.matrix = matrixd(1,vector<double>(5,0.0));
    waypoints.matrix = {{waypoint_x,waypoint_y,waypoint_z,waypoint_yaw,waypoint_time}};
    matrixds old_waypoints = quadrotor.get_waypoints();
    if ((old_waypoints.matrix[old_waypoints.l - 1 ][4]) < waypoint_time){
         quadrotor.set_waypoints(waypoints);
         modifier->creat_sphere(waypoint_x,waypoint_y,waypoint_z);
    }
        else{
         cout << "Invalid time waypoint" << endl;
         QMessageBox msgBox;
         msgBox.setWindowTitle("Warning");
         msgBox.setText("Invalid TIME waypoint");
         msgBox.exec();
    }
}

void mainwindow::init_3dquad(){



   //root entity
   Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

   //Camera
   cameraEntity = view->camera();
   cameraEntity->lens()->setPerspectiveProjection(45.0f,16.0f/9.0f,0.1f,1000.0f);
   cameraEntity->setPosition(QVector3D(4,4,2.5f));
   cameraEntity->setUpVector(QVector3D(0,0,1));
   cameraEntity->setViewCenter(QVector3D(-3,-3,0));



   Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
   Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
   light->setColor("white");
   light->setIntensity(2);
   lightEntity->addComponent(light);
   Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
   lightTransform->setTranslation(QVector3D(0,0,20.0f));
   lightEntity->addComponent(lightTransform);

   //For camera controls
   Qt3DExtras::QFirstPersonCameraController *camController = new Qt3DExtras::QFirstPersonCameraController(rootEntity);
   camController->setCamera(cameraEntity);

   modifier = new scenemodifier(rootEntity);
   modifier->set_params(quadrotor.get_params());

   matrixds waypoints =quadrotor.get_waypoints();

   for(int i = 1; i < waypoints.l; i++ ){

       modifier->creat_sphere(waypoints.matrix[i][0],waypoints.matrix[i][1],waypoints.matrix[i][2]);

   }

   view->setRootEntity(rootEntity);



}

void mainwindow::on_reset_quad_clicked(){
    quadrotor.init_Quad();
    init_3dquad();
    ui->LcdTempo->display(0);
}

void mainwindow::on_resetway_clicked(){
    quadrotor.init_waypoints();
    quadrotor.init_Quad();
    init_3dquad();
    ui->LcdTempo->display(0);
}

void mainwindow::on_change_controller_clicked(){
    string quad_controller_decision = ui->controller_options->currentText().toUtf8().constData();
    if(quad_controller_decision == "Linear"){
        quadrotor.set_controller(1);
    }
    else if(quad_controller_decision == "Thrust up"){
        quadrotor.set_controller(2);
    }
    else if(quad_controller_decision == "Geometric Tracking"){
        quadrotor.set_controller(3);
    }
    else{
        quadrotor.set_controller(3);
    }
}

void mainwindow::on_optimize_gain_clicked(){
    optimization.set_params(quadrotor.get_params());
    optimization.set_waypoints(quadrotor.get_waypoints());
    string quad_controller_decision = ui->controller_options->currentText().toUtf8().constData();

    if(quad_controller_decision == "Linear"){
        optimization.set_control(1);
    }
    else if(quad_controller_decision == "Thrust up"){
        optimization.set_control(2);
    }
    else if(quad_controller_decision == "Geometric Tracking"){
        optimization.set_control(3);
    }
    else{
        optimization.set_control(3);
    }

    optimization.optimize();

}
