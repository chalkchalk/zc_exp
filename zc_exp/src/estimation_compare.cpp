#include "estimation_compare.h"
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include "eigenmvn.h"

SensorNodeCompare::SensorNodeCompare()
{
    index = 0;
    pos_now = Eigen::Vector3d(1e5, 1e5, 1e5);
    pos_leader = Eigen::Vector3d(1e5, 1e5, 1e5);
    delta_q.setZero();
    delta_omega.setZero();
    delta_qc.setZero();
    delta_omegac.setZero();
    delta_q0c.setZero();
    omega0_c.setZero();
    hat_r0.setZero();
    hat_r1.setZero();
    P_0.setIdentity();
    omega_0 = P_0.inverse();
    omega_1.setZero();
    q_0 = omega_0 * hat_r0;
    q_1.setZero();
    q_0c.setZero();
    Rs = 8.0; // 6.0
    pos_follow = Eigen::Vector2d(1e5, 1e5);
    R = 0.00001 * Eigen::Matrix3d::Identity();
    Q = 0.00001 * Eigen::Matrix3d::Identity();
    dt = INTERVAL_MS * 0.001;
    start_attack = false;
}

void SensorNodeCompare::propagation_step(int time_k)
{
    hat_r0(0) = Utils::periodical_clamp(hat_r1(0) + vel_leader(0) * dt, -PI, PI, 2 * PI);
    // hat_r0(3*i-2,k+1)=mod(hat_r1(3*i-2,k)+omega,2*pi);
    Eigen::Matrix2d hat_R;
    hat_R << cos(hat_r1(0)), -sin(hat_r1(0)),
        sin(hat_r1(0)), cos(hat_r1(0));
    hat_r0.block<2, 1>(1, 0) = hat_r1.block<2, 1>(1, 0) + hat_R * vel_leader.block<2, 1>(1, 0) * dt;
    // hat_r0(3*i-1:3*i,k+1)=hat_r1(3*i-1:3*i,k)+[cos(hat_r1(1,k)),-sin(hat_r1(1,k));sin(hat_r1(1,k)),cos(hat_r1(1,k))]*v;
    omega_0 = Q.inverse() + Q.inverse() * (omega_1 + Q.inverse()).inverse() * Q.inverse();
    // Omega_0(3*i-2:3*i,3*k+1:3*k+3)=inv(Q)+inv(Q)*inv(Omega_1(3*i-2:3*i,3*k-2:3*k)+inv(Q))*inv(Q);
    q_0 = omega_0 * hat_r0;
    // q_0(3*i-2:3*i,k+1)=Omega_0(3*i-2:3*i,3*k+1:3*k+3)*hat_r0(3*i-2:3*i,k+1);

    std::cout << index << ":" << hat_r1.transpose() << std::endl;
    if (pose_history.size > 0)
    {
        if ((pose_history.get_data(0).block<2, 1>(1, 0) - hat_r1.block<2, 1>(1, 0)).norm() > 0.02)
        {
            pose_history.append(hat_r1);
        }
    }
    else
    {
        pose_history.append(hat_r1);
    }

    if (pose_history.size > 25 * (index + 1))
    {
        pos_follow = pose_history.get_data(25 * (index + 1)).block<2, 1>(1, 0);
    }
}

void SensorNodeCompare::perceive_setp(int time_k)
{
    if ((pos_leader.block<2, 1>(1, 0) - pos_now.block<2, 1>(1, 0)).norm() > Rs)
    {
        delta_q.setZero();
        delta_omega.setZero();
        // delta_q(3*i-2:3*i,k)=zeros(3,1);
        // delta_omega(3*i-2:3*i,3*k-2:3*k)=zeros(3,3);
    }
    else
    {
        Eigen::Matrix3d H;
        Eigen::Vector3d y;
        H << 1, 0, 0,
            0, cos(pos_now(2)), sin(pos_now(2)),
            0, -sin(pos_now(2)), cos(pos_now(2));
        delta_omega = H.transpose() * R.inverse() * H;
        // delta_omega(3*i-2:3*i,3*k-2:3*k)=H(3*i-2:3*i,3*k-2:3*k)'*inv(R)*H(3*i-2:3*i,3*k-2:3*k);

        if (!start_attack)
        {
            y = H * pos_leader + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
        }
        else
        {
            if (time_k % 8 == 0)
            {
                if (index == 1)
                { // 2
                    y = 3 * H * pos_leader - 2 * pos_now + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
                }
                else
                {
                    y = H * pos_leader  + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
                }
            }
            else if (time_k % 8 == 4)
            {
                    if (index == 2)
                    { // 3
                        y = 3 * H * pos_leader - 2 * pos_now + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
                    }
                    else
                    {
                         y = H * pos_leader  + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
                    }
            }
            else
            {
                 y = H * pos_leader  + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
            }
        }
        delta_q = H.transpose() * R.inverse() * y;
    }
}

void SensorNodeCompare::update_step(int time_k)
{
    omega_1 = omega0_c + 5 * delta_omegac;
    q_1 = q_0c + 5 * delta_qc;
    hat_r1 = omega_1.inverse() * q_1;
    // Omega_1(3*i-2:3*i,3*k-2:3*k)=Omega_0c(3*i-2:3*i,3*k-2:3*k)+5*delta_omegac(3*i-2:3*i,3*k-2:3*k);%计算本地后验信息矩阵
    // q_1(3*i-2:3*i,k)=q_0c(3*i-2:3*i,k)+5*delta_qc(3*i-2:3*i,k);%计算本地后验信息向量
    // hat_r1(3*i-2:3*i,k)=inv(Omega_1(3*i-2:3*i,3*k-2:3*k))*q_1(3*i-2:3*i,k);%后验估计值
}

EstimationSystemCompare::EstimationSystemCompare()
{
    A1 << 0.4, 0, 0.3, 0.3, 0,
        0, 0.5, 0, 0.25, 0.25,
        0.3, 0, 0.5, 0.2, 0,
        0.3, 0.25, 0.2, 0.25, 0,
        0, 0.25, 0, 0.25, 0.5;
    A2 << 0.5, 0.25, 0.25, 0, 0,
        0.25, 0.25, 0, 0.25, 0.25,
        0.25, 0, 0.6, 0.15, 0,
        0, 0.25, 0.15, 0.4, 0.2,
        0, 0.25, 0, 0.2, 0.55;
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].index = i;
    }
}

void EstimationSystemCompare::update_leader_pose_vel(Eigen::Vector3d pos, Eigen::Vector3d vel)
{
    pos_leader = pos;
    vel_leader = vel;
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].pos_leader = pos;
        sensors[i].vel_leader = vel;
    }
}

void EstimationSystemCompare::estimation_step(Eigen::Vector3d pos_leader, Eigen::Vector3d vel)
{
    if (pos_leader.norm() + vel.norm() > 200)
    {
        return;
    }
    update_leader_pose_vel(pos_leader, vel);
    perceive_setp();
    consensus_step();
    update_step();
    propagation_step();
    time_k++;
}

void EstimationSystemCompare::consensus_step()
{
    Eigen::Matrix<double, NUM_OF_SENSOR, NUM_OF_SENSOR> A;
    A = time_k % 2 ? A2 : A1;
    Eigen::Matrix<double, NUM_OF_SENSOR * 3, 1> delta_q_all;
    Eigen::Matrix<double, NUM_OF_SENSOR * 3, 3> delta_omega_all;
    Eigen::Matrix<double, NUM_OF_SENSOR * 3, 3> omega_0_all;
    Eigen::Matrix<double, NUM_OF_SENSOR * 3, 1> q_0_all;
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        delta_q_all.block<3, 1>(i * 3, 0) = sensors[i].delta_q;
        delta_omega_all.block<3, 3>(i * 3, 0) = sensors[i].delta_omega;
        omega_0_all.block<3, 3>(i * 3, 0) = sensors[i].omega_0;
        q_0_all.block<3, 1>(i * 3, 0) = sensors[i].q_0;
    }
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].delta_qc = Eigen::kroneckerProduct(A.block<1, NUM_OF_SENSOR>(i, 0), Eigen::Matrix3d::Identity()) * delta_q_all;
        // delta_qc(3*i-2:3*i,k)=kron(A(i,:),eye(3))*delta_q(:,k);%计算融合后的更新量
        sensors[i].delta_omegac = Eigen::kroneckerProduct(A.block<1, NUM_OF_SENSOR>(i, 0), Eigen::Matrix3d::Identity()) * delta_omega_all;
        // delta_omegac(3*i-2:3*i,3*k-2:3*k)=kron(A(i,:),eye(3))*delta_omega(:,3*k-2:3*k);%计算融合后的更新量
        sensors[i].omega0_c = Eigen::kroneckerProduct(A.block<1, NUM_OF_SENSOR>(i, 0), Eigen::Matrix3d::Identity()) * omega_0_all;
        // Omega_0c(3*i-2:3*i,3*k-2:3*k)=kron(A(i,:),eye(3))*Omega_0(:,3*k-2:3*k);
        sensors[i].q_0c = Eigen::kroneckerProduct(A.block<1, NUM_OF_SENSOR>(i, 0), Eigen::Matrix3d::Identity()) * q_0_all;
        // q_0c(3*i-2:3*i,k)=kron(A(i,:),eye(3))*q_0(:,k);
    }
}

void EstimationSystemCompare::perceive_setp()
{
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].perceive_setp(time_k);
    }
}

void EstimationSystemCompare::update_step()
{
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].update_step(time_k);
    }
}

void EstimationSystemCompare::propagation_step()
{
    for (int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].start_attack = start_attack;
        sensors[i].propagation_step(time_k);
    }
}
