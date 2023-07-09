#ifndef ESTIMATION_COMPARE_H_
#define ESTIMATION_COMPARE_H_

#include <Eigen/Dense>
#include "utils.h"

#define NUM_OF_SENSOR 5
#define INTERVAL_MS 50.0

class SensorNodeCompare
{
public:
    SensorNodeCompare();
    int index;
    Eigen::Vector3d pos_now;
    Eigen::Vector3d pos_leader;
    Eigen::Vector3d vel_leader;
    double Rs; //detect range
    Eigen::Vector3d delta_q;
    Eigen::Matrix3d delta_omega;
    Eigen::Vector3d delta_qc;
    Eigen::Matrix3d delta_omegac;
    Eigen::Vector3d delta_q0c;
    Eigen::Matrix3d omega0_c;
    Eigen::Matrix3d omega_0;
    Eigen::Matrix3d omega_1;
    Eigen::Vector3d q_0;
    Eigen::Vector3d q_1;
    Eigen::Vector3d q_0c;
    Eigen::Matrix3d P_0;
    Eigen::Vector3d hat_r0;
    Eigen::Vector3d hat_r1;
    void perceive_setp(int time_k);
    void update_step(int time_k);
    void propagation_step(int time_k);
    Eigen::Matrix3d R;
    Eigen::Matrix3d Q;
    Queue<Eigen::Vector3d> pose_history;
    Eigen::Vector2d pos_follow;
    double dt;
    bool start_attack;
    
};

class EstimationSystemCompare
{
public:
    EstimationSystemCompare();
    void estimation_step(Eigen::Vector3d pos_leader, Eigen::Vector3d vel);
    SensorNodeCompare sensors[NUM_OF_SENSOR];
    bool start_attack;
private:
    int time_k;
    Eigen::Vector3d pos_leader;
    Eigen::Vector3d vel_leader;
    Eigen::Matrix<double, NUM_OF_SENSOR, NUM_OF_SENSOR> A1;
    Eigen::Matrix<double, NUM_OF_SENSOR, NUM_OF_SENSOR> A2;

    void update_leader_pose_vel(Eigen::Vector3d pos, Eigen::Vector3d vel);
    void perceive_setp();
    void consensus_step();
    void update_step();
    void propagation_step();

};


#endif
