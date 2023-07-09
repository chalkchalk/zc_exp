#ifndef ESTIMATION_H_
#define ESTIMATION_H_

#include <Eigen/Dense>
#include "utils.h"

#define NUM_OF_SENSOR 5
#define INTERVAL_MS 50.0

class SensorNode
{
public:
    SensorNode();
    int index;
    Eigen::Vector3d pos_now;
    Eigen::Vector3d pos_leader;
    Eigen::Vector3d vel_leader;
    Eigen::Vector3d hat_r_ic;
    Eigen::Vector3d hat_r1; // theta, x, y
    Eigen::Matrix3d P_ic;
    Eigen::Matrix3d P_0;
    Eigen::Matrix3d P_1;
    Eigen::Matrix3d R;
    Eigen::Matrix3d Q;
    Eigen::Vector3d hat_r0;
    double Rs; //detect range
    double psi_0;
    double tau_psi;
    double dt;
    void update_step(int time_k);
    void propagation_step(int time_k);
    Queue<Eigen::Vector3d> pose_history;
    Eigen::Vector2d pos_follow;
};

class EstimationSystem
{
public:
    EstimationSystem();
    void estimation_step(Eigen::Vector3d pos_leader, Eigen::Vector3d vel);
    SensorNode sensors[NUM_OF_SENSOR];
private:
    Eigen::Matrix<double, NUM_OF_SENSOR, NUM_OF_SENSOR> A1;
    Eigen::Matrix<double, NUM_OF_SENSOR, NUM_OF_SENSOR> A2;
    Eigen::Vector3d pos_leader;
    Eigen::Vector3d vel_leader;
    double alpha_0;
    double tau_alpha;
    int time_k;
    double get_alpha_k();
    void consensus_step();
    void update_step();
    void propagation_step();
    void update_leader_pose_vel(Eigen::Vector3d pos, Eigen::Vector3d vel);

};


#endif
