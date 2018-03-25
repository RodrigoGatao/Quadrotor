#include "quad.h"
#include "math.h"
#include "utils.h"
#include "iostream"
//#include"windows.h"
#define PI  3.141516
using namespace std;

//Constructor
quad::quad()
{
    controlhandle = new controller;
    init_Quad();
    init_params();
    init_waypoints();
//    waypoints.l = 12;

//    waypoints.matrix = { {0, 0, 0, 0, 0},
//                         {-0.4, 0, 1, 0, 2},
//                         {0, 0, 1, 0, 3},
//                         {0.2, 0, 1.6, 0, 4},
//                         {0.4, 0.4, 1, 1.57, 5},
//                         {0.6, 0, 0.4, 3.14, 5.8},
//                         {0.8, -0.4, 1, 4.71, 6.6},
//                         {1, 0, 1.6, 6.28, 7.4},
//                         {1.2, 0.4, 1, 7.85, 8.2},
//                         {1.4, 0, 0.4, 9.42, 9},
//                         {1.6, -0.4, 1, 10.99, 9.8},
//                         {1.8, 0, 1.6, 12.56, 10.6}};
        waypoints.l = 3;
//        waypoints.matrix = {{0   ,0   ,0  ,0    ,0},
//                                {0.5 ,1   ,1  ,0    ,1},
//                                {1   ,0   ,2  ,0    ,2},
//                                {1.5 ,-1  ,1  ,0    ,3},
//                                {2   ,0   ,0  ,0    ,4}};
        waypoints.matrix = {{0,0,0,0,0},
                                {2,-1,1,3.14,1.5},
                                {-1,2,2,3.14,3}};
    controlhandle->set_waypoints(waypoints);
}

//Destructor
quad::~quad()
{

}

void quad::run(){
    while(is_running){
        old_des_state = des_state;
        old_state = state;
        des_state = controlhandle->trajhandle(t);
        motor = controlhandle->update_motor(t,state);
        model();
        emit emit_quadStates(state,old_state,des_state,old_des_state,t);
        msleep(quadparams.dt * 1000);
    }
}

/*****************
 *  Miscelanius Functions *
 ****************/
void quad::init_params(){
    setParams(1,0.468);
    setParams(2,0.225);
    setParams(3,1.14*pow(10,-7));
    setParams(4,2.98*pow(10,-6));
    setParams(5,0.004856);
    setParams(6,0.004856);
    setParams(7,0.008801);
    setParams(8,9.81);
    setParams(9,0.02);
    controlhandle->set_params(quadparams.mass,quadparams.dt,quadparams.gravity,quadparams.Ixx,quadparams.Iyy,quadparams.Izz,quadparams.k,quadparams.b,quadparams.l);

}

void quad::init_Quad()
{


    position = receive_matrix(1,3);
    orientation = receive_matrix(1,3);
    linear_vel = receive_matrix(1,3);
    linear_acc = receive_matrix(1,3);
    angular_vel = receive_matrix(1,3);
    angular_acc = receive_matrix(1,3);
    angular_vel_quad = receive_matrix(1,3);
    state = receive_matrix(4,3);
    old_state = receive_matrix(4,3);
    des_state = receive_matrix(5,3);
    old_des_state = receive_matrix(5,3);
    motor = receive_matrix(1,4);
    b3 = receive_matrix(1,3);
    b3.matrix = {{0,0,1}};
    t = 0;
    iteration = 0;


}

void quad::init_waypoints(){

    waypoints = receive_matrix(100,5);
    waypoints.l = 1;
    controlhandle->set_waypoints(waypoints);

}

void quad::model(){
    double roll = orientation.matrix[0][0];
    double pitch = orientation.matrix[0][1];
    double yaw = orientation.matrix[0][2];

    matrixds vectora, vectorb;
    vectora = receive_matrix(1,3);
    vectorb = receive_matrix(1,3);

    R = rotation_matrix(roll,pitch,yaw);

    double thrust = quadparams.k*(motor.matrix[0][0] + motor.matrix[0][1] + motor.matrix[0][2] + motor.matrix[0][3])/quadparams.mass;
   // thrust = motor.matrix[0][0]/quadparams.mass;

    linear_acc = sum_matrix(multiple_matrix((-quadparams.gravity),b3),transposed_matrix(multiple_matrix(thrust,column_matrix(R,2))));
    linear_vel = sum_matrix(linear_vel,multiple_matrix(quadparams.dt,linear_acc));
    position = sum_matrix(position,multiple_matrix(quadparams.dt,linear_vel));

    /*****************************************************************************************************/
    double p = angular_vel_quad.matrix[0][0];
    double q = angular_vel_quad.matrix[0][1];
    double r = angular_vel_quad.matrix[0][2];

    vectora.matrix = {{((quadparams.Iyy-quadparams.Izz)*q*r/quadparams.Ixx),
                       ((quadparams.Izz-quadparams.Ixx)*p*r/quadparams.Iyy),
                       ((quadparams.Ixx-quadparams.Iyy)*p*q/quadparams.Izz)}};
    vectorb.matrix = {{(quadparams.l*quadparams.k*((motor.matrix[0][1])-(motor.matrix[0][3]))/quadparams.Ixx),
                       (quadparams.l*quadparams.k*(-(motor.matrix[0][0])+(motor.matrix[0][2]))/quadparams.Iyy),
                       (quadparams.b*((motor.matrix[0][0])-(motor.matrix[0][1])+(motor.matrix[0][2])-(motor.matrix[0][3]))/quadparams.Izz)}};

    angular_acc = sum_matrix(vectora,vectorb);
    /*****************************************************************************************************/

//    matrixds I = receive_matrix(3,3), vec = receive_matrix(3,1);
//    I.matrix = {{quadparams.Ixx,0,0},
//         {0,quadparams.Iyy,0},
//         {0,0,quadparams.Izz}};
//    vec.matrix = {{motor.matrix[0][1]},{motor.matrix[0][2]},{motor.matrix[0][3]}};
//    angular_acc = transposed_matrix(product_matrix(inverse_matrix(I),vec));

    angular_vel_quad = sum_matrix(angular_vel_quad,multiple_matrix(quadparams.dt,angular_acc));
    angular_vel = transposed_matrix(product_matrix(inv_transformation_matrix(roll,pitch,yaw),transposed_matrix(angular_vel_quad)));
    orientation = sum_matrix(orientation,multiple_matrix(quadparams.dt,angular_vel));

    if(orientation.matrix[0][0] > PI){
        orientation.matrix[0][0] = orientation.matrix[0][0] - 2*PI;
    }
    if(orientation.matrix[0][0] < -PI){
        orientation.matrix[0][0] = orientation.matrix[0][0] + 2*PI;
    }
    if(orientation.matrix[0][1] > PI){
        orientation.matrix[0][1] = orientation.matrix[0][1] - 2*PI;
    }
    if(orientation.matrix[0][1] < -PI){
        orientation.matrix[0][1] = orientation.matrix[0][1] + 2*PI;
    }

    old_state = state;
    state.matrix[0] = {{position.matrix[0][0], position.matrix[0][1], position.matrix[0][2]}};
    state.matrix[1] = {{linear_vel.matrix[0][0], linear_vel.matrix[0][1], linear_vel.matrix[0][2]}};
    state.matrix[2] = {{orientation.matrix[0][0], orientation.matrix[0][1], orientation.matrix[0][2]}};
    state.matrix[3] = {{angular_vel_quad.matrix[0][0], angular_vel_quad.matrix[0][1], angular_vel_quad.matrix[0][2]}};

    t = t + quadparams.dt;
    iteration++;
}

/*****************
 *  Get Functions *
 ****************/

matrixds quad::get_waypoints(){
    return waypoints;
}

params quad::get_params(){
    return quadparams;
}

/*****************
 *  Set Functions *
 ****************/
void quad::set_controller(int a){
    controlhandle->set_controller(a);
}

void quad::setParams(int select, double value){
    switch (select) {
    case 1:
       quadparams.mass = value;
        break;
    case 2:
       quadparams.l = value;
        break;
    case 3:
       quadparams.b = value;
        break;
    case 4:
       quadparams.k = value;
        break;
    case 5:
       quadparams.Ixx = value;
        break;
    case 6:
       quadparams.Iyy = value;
        break;
    case 7:
       quadparams.Izz = value;
        break;
    case 8:
       quadparams.gravity = value;
        break;
    case 9:
       quadparams.dt = value;
        break;
    default:
        break;
    }
    controlhandle->set_params(quadparams.mass,quadparams.dt,quadparams.gravity,quadparams.Ixx,quadparams.Iyy,quadparams.Izz,quadparams.k,quadparams.b,quadparams.l);

};

void quad::set_run(bool a){
    is_running = a;
}

void quad::set_waypoints(matrixds matrix){
    waypoints.matrix[waypoints.l] = {{matrix.matrix[0][0],matrix.matrix[0][1],matrix.matrix[0][2],matrix.matrix[0][3],matrix.matrix[0][4]}};
    waypoints.l++;
    //Abaixo o objeto da classe controller envia o parametro waypoints
    controlhandle->set_waypoints(waypoints);
}










