#include "pso.h"
#include"iostream"

using namespace std;


pso::pso()
{
    controlhandle = new controller;
    waypoints = receive_matrix(100,5);
    waypoints.l = 1;
    n = 70;
    srand(time(NULL));
    alpha = 0.03;
    beta = 0.0003;
    ksi = 0.1;
    t_max = 10;

}

double pso::fob(matrixds gains){

    controlhandle->set_controller(control);
        if(control == 1){
            controlhandle->set_l_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3], gains.matrix[0][4], gains.matrix[0][5]);
        }
        else if(control == 2){
            controlhandle->set_tu_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3]);
        }
        else if(control == 3){
            controlhandle->set_gt_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3]);
        }
        else{
            controlhandle->set_gt_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3]);
        }
        //cout << "control" << control << endl;
        double erro = 0;
        double t = 0;
        matrixds position = receive_matrix(1,3);
        matrixds orientation = receive_matrix(1,3);
        matrixds linear_vel = receive_matrix(1,3);
        matrixds linear_acc = receive_matrix(1,3);
        matrixds angular_vel = receive_matrix(1,3);
        matrixds angular_vel_quad = receive_matrix(1,3);
        matrixds angular_acc = receive_matrix(1,3);
        matrixds b3 = receive_matrix(1,3);
        b3.matrix = {{0,0,1}};
        matrixds state = receive_matrix(4,3);
        matrixds des_state = receive_matrix(5,3);
        matrixds motor = receive_matrix(1,4);

        int vmax = (waypoints.matrix[waypoints.l - 1][4]/quadparams.dt) + 10;
        //cout << "vmax" << vmax << endl;
        for(int i = 0; i < vmax; i++){
            des_state = controlhandle->trajhandle(t);
            motor = controlhandle->update_motor(t,state);

            double roll = orientation.matrix[0][0];
            double pitch = orientation.matrix[0][1];
            double yaw = orientation.matrix[0][2];

            matrixds vectora, vectorb;
            vectora = receive_matrix(1,3);
            vectorb = receive_matrix(1,3);

            matrixds R = rotation_matrix(roll,pitch,yaw);

            double thrust = quadparams.k*(motor.matrix[0][0]+motor.matrix[0][1]+motor.matrix[0][2]+motor.matrix[0][3])/quadparams.mass;

            linear_acc = sum_matrix(multiple_matrix((-quadparams.gravity),b3),transposed_matrix(multiple_matrix(thrust,column_matrix(R,2))));
            linear_vel = sum_matrix(linear_vel,multiple_matrix(quadparams.dt,linear_acc));
            position = sum_matrix(position,multiple_matrix(quadparams.dt,linear_vel));

            double p = angular_vel_quad.matrix[0][0];
            double q = angular_vel_quad.matrix[0][1];
            double r = angular_vel_quad.matrix[0][2];

            vectora.matrix = {{((quadparams.Iyy-quadparams.Izz)*q*r/quadparams.Ixx),
                               ((quadparams.Izz-quadparams.Ixx)*p*r/quadparams.Iyy),
                               ((quadparams.Ixx-quadparams.Iyy)*p*q/quadparams.Izz)}};
            vectorb.matrix = {{(quadparams.l*quadparams.k*(motor.matrix[0][1]-motor.matrix[0][3])/quadparams.Ixx),
                               (quadparams.l*quadparams.k*(-motor.matrix[0][0]+motor.matrix[0][2])/quadparams.Iyy),
                               (quadparams.b*(motor.matrix[0][0]-motor.matrix[0][1]+motor.matrix[0][2]-motor.matrix[0][3])/quadparams.Izz)}};

            angular_acc = sum_matrix(vectora,vectorb);

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

            state.matrix[0] = {{position.matrix[0][0], position.matrix[0][1], position.matrix[0][2]}};
            state.matrix[1] = {{linear_vel.matrix[0][0], linear_vel.matrix[0][1], linear_vel.matrix[0][2]}};
            state.matrix[2] = {{orientation.matrix[0][0], orientation.matrix[0][1], orientation.matrix[0][2]}};
            state.matrix[3] = {{angular_vel_quad.matrix[0][0], angular_vel_quad.matrix[0][1], angular_vel_quad.matrix[0][2]}};

            t = t + quadparams.dt;
            erro = erro + pow((state.matrix[0][0]-des_state.matrix[0][0]),2) + pow((state.matrix[0][1]-des_state.matrix[0][1]),2) + pow((state.matrix[0][2]-des_state.matrix[0][2]),2) + pow((state.matrix[2][2]-des_state.matrix[3][0]),2);
        }
        return erro;

}

void pso::set_waypoints(matrixds waypoints){

    this->waypoints = waypoints;
    controlhandle->set_waypoints(waypoints);
}

void pso::set_params(params quadparams){
    this->quadparams = quadparams;
    controlhandle->set_params(quadparams.mass,quadparams.dt,quadparams.gravity,quadparams.Ixx,quadparams.Iyy,quadparams.Izz,quadparams.k,quadparams.b,quadparams.l);

}

void pso::set_control(int a){
       control = a;
}

void pso::set_range_gains(){
    if (control == 1){
        gains_min.resize(1,6);
        gains_min << 0,0,0,0,0,0;
        gains_max.resize(1,6);
        gains_max <<50,50,200,200,1000,100;
    }
    else if (control == 2){
        gains_min.resize(1,4);
        gains_min << 0,0,0,0;
        gains_max.resize(1,4);
        gains_max <<100,1,1000,1;
    }
    else if (control == 3){
        gains_min.resize(1,4);
        gains_min << 0,0,0,0;
        gains_max.resize(1,4);
        gains_max <<50,10,50,1;
    }
    else{
        gains_min.resize(1,4);
        gains_min << 0,0,0,0;
        gains_max.resize(1,4);
        gains_max <<50,10,50,1;
    }
}

void pso::optimize(){

    int t_T = 0;

    set_range_gains();
    //cout << "gains" << gains_min.cols() << endl;
    MatrixXd pos;
    pos.resize(n,gains_min.cols());
    for(int i = 0; i < n; i++ ){
        for(int j = 0; j < gains_min.cols(); j++){
            pos(i,j) = {(gains_max(0,j) - gains_min(0,j)) * (double)rand()/RAND_MAX + gains_min(0,j)};
        }
    }

    //cout << "pos" << pos.row(1) <<endl;

    MatrixXd vels;
    vels = MatrixXd::Zero(n,gains_min.cols());
    //cout << "vels" << vels << endl;

    MatrixXd fitness;
    fitness.resize(n,1);
    for(int i = 0; i < n; i++){
        fitness(i,0) = fob(mxd2mds(pos.row(i)));
    }
    //cout << "fitness" << fitness << endl;
    MatrixXd pos_star;
    pos_star.resize(n,gains_min.cols());
    pos_star = pos;

    MatrixXd fitness_star;
    fitness_star.resize(n,1);
    fitness_star = fitness;

    double best_fitness = fitness.minCoeff();
    //cout << "best" << best_fitness << endl;

    MatrixXd best_pos;
    best_pos.resize(1,gains_min.cols());
    for (int i = 0; i < n; i++){
        if(fitness_star(i,0) == best_fitness){
            best_pos = pos_star.row(i);
        }
    }
    //cout << "ganhos" << pos.row(index) << endl;

    //cout << "qualquer coisa" << best_fitness << "1" << ksi << "2" << t_T << "3" << t_max << endl;
    while(best_fitness > ksi && t_T<t_max){
        t_T++;
        for(int i = 0; i < n; i++ ){
            for(int j =0; j < gains_min.cols(); j++){
                vels(i,j) = vels(i,j) + alpha * (best_pos(0,j) - pos(i,j))*(double)rand()/RAND_MAX + beta* (pos_star(i,j) - pos(i,j))*(double)rand()/RAND_MAX;
            }
        }
    pos = pos + vels;
    for(int i = 0; i < n; i++){
        for(int j = 0; j <gains_min.cols(); j++){
            if(pos(i,j) < gains_min(0,j)){
                pos(i,j) = gains_min(0,j);
            }
            if(pos(i,j) > gains_max(0,j)){
                pos(i,j) = gains_max(0,j);
            }
        }
    }
    for(int i = 0; i < n; i++){
        fitness(i,0) = fob(mxd2mds(pos.row(i)));
    }
    for(int i = 0; i < n; i++){
        if(fitness(i,0) < fitness_star(i,0)){
            fitness_star(i,0) = fitness(i,0);
            pos_star.row(i) = pos.row(i);
        }
    }
    best_fitness = fitness_star.minCoeff();
    for (int i = 0; i < n; i++){
        if(fitness_star(i,0) == best_fitness){
            best_pos = pos_star.row(i);
        }
    }
    cout << "t" << t_T << endl;
    }
    cout << "erro" << best_fitness << endl;
    cout << "pos" << best_pos << endl;
}
