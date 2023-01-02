#include "estimation.h"
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include "eigenmvn.h"


SensorNode::SensorNode()
{
    index = 0;
    pos_now = Eigen::Vector3d(1e5, 1e5, 1e5);
    pos_leader = Eigen::Vector3d(1e5, 1e5, 1e5);
    hat_r_ic.setZero();
    hat_r0.setZero();
    hat_r1.setZero();
    P_1.setIdentity();
    P_ic.setIdentity();
    P_0.setIdentity();
    R = 0.00001 * Eigen::Matrix3d::Identity();
    Q = 0.00001 * Eigen::Matrix3d::Identity();
    Rs = 6.0;
    psi_0 = 0.7;
    tau_psi = 0.8;
    pos_follow = Eigen::Vector2d(1e5, 1e5);
    dt = INTERVAL_MS * 0.001;
}

void SensorNode::update_step(int time_k)
{
    // if norm(r(2:3,k)-s(3*i-1:3*i,k),2)>Rs %不能获得检测值的情况下计算后验估计
    if((pos_leader.block<2, 1>(1, 0) - pos_now.block<2, 1>(1, 0)).norm() > Rs)
    {
        hat_r1 = hat_r_ic;
        P_1 = P_ic;
        // hat_r1(3*i-2:3*i,k)= hat_r_ic(3*i-2:3*i,k);
        // P_1(3*i-2:3*i,3*k-2:3*k)=P_ic(3*i-2:3*i,3*k-2:3*k);
    }
    else
    {
        Eigen::Matrix3d H;
        Eigen::Vector3d z;
        H << 1, 0, 0,
            0, cos(pos_now(2)), sin(pos_now(2)),
            0, -sin(pos_now(2)), cos(pos_now(2));
        //H(3*i-2:3*i,3*k-2:3*k)=[1,0,0;0,cos(s(3*i-2,k)),sin(s(3*i-2,k));0,-sin(s(3*i-2,k)),cos(s(3*i-2,k))];%观测矩阵
        if (time_k % 8 == 0)
        {
            if(index == 2)
            {
                z = 2 * (H * (pos_leader - pos_now)) + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
            }
            else
            {
                z = (H * (pos_leader - pos_now)) + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
            }
            // if i==2
            //     z(3*i-2:3*i,k)=2*(H(3*i-2:3*i,3*k-2:3*k)*(r(:,k)-s(3*i-2:3*i,k)))+mvnrnd([0;0;0]',R,1)';%被攻击后的观测值
            // else
            //     z(3*i-2:3*i,k)=H(3*i-2:3*i,3*k-2:3*k)*(r(:,k)-s(3*i-2:3*i,k))+mvnrnd([0;0;0]',R,1)';%观测值
            // end
        }
        else
        {
            if (time_k % 8 == 4)
            {
                if (index == 1)
                {
                    z = 3 * (H * (pos_leader - pos_now)) + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
                }
                else
                {
                    z = (H * (pos_leader - pos_now)) + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
                }
            }
            else
            {
                z = (H * (pos_leader - pos_now)) + Eigen::EigenMultivariateNormal<double>(Eigen::Vector3d::Zero(), R).samples(1);
            }
        }
        Eigen::Matrix3d S = H * P_ic * H.transpose() + R;
        Eigen::Matrix3d K = P_ic * H.transpose() * S.inverse();
        // S(3*i-2:3*i,3*k-2:3*k)=H(3*i-2:3*i,3*k-2:3*k)*P_ic(3*i-2:3*i,3*k-2:3*k)*H(3*i-2:3*i,3*k-2:3*k)'+R;
        // K(3*i-2:3*i,3*k-2:3*k)=P_ic(3*i-2:3*i,3*k-2:3*k)*H(3*i-2:3*i,3*k-2:3*k)'*inv(S(3*i-2:3*i,3*k-2:3*k));%kalman增益
        P_1 = P_ic - K * H * P_ic;
        Eigen::Vector3d delta_h = z - H * (hat_r_ic - pos_now);
        double psi_k = psi_0 / pow(time_k + 1, tau_psi);
        // P_1(3*i-2:3*i,3*k-2:3*k)=P_ic(3*i-2:3*i,3*k-2:3*k)-K(3*i-2:3*i,3*k-2:3*k)*H(3*i-2:3*i,3*k-2:3*k)*P_ic(3*i-2:3*i,3*k-2:3*k);
        // delta_h(3*i-2:3*i,k)=z(3*i-2:3*i,k)-H(3*i-2:3*i,3*k-2:3*k)*(hat_r_ic(3*i-2:3*i,k)-s(3*i-2:3*i,k));%观测残差
        // psi(k)=psi_0*inv((k+1)^tau_psi);%计算时变邻域值\psi_k
        double Phi_k;
        if(delta_h.norm() <= psi_k)
        {
            Phi_k = 1;
        }
        else
        {
            Phi_k = psi_k / delta_h.norm();
        }
        // if norm(delta_h(3*i-2:3*i,k),2)<=psi(k)
        //     Phi(i,k)=1;
        // else
        //     Phi(i,k)=psi(k)*inv(norm(delta_h(3*i-2:3*i,k),2));
        // end
        Eigen::Vector3d delta_r = K * Phi_k * delta_h;
        // delta_r(3*i-2:3*i,k)= K(3*i-2:3*i,3*k-2:3*k)*Phi(i,k)*delta_h(3*i-2:3*i,k);%增益乘残差
        hat_r1(0) = Utils::periodical_clamp(hat_r_ic(0) + delta_r(0), -PI, PI, 2 * PI);
        // hat_r1(3*i-2,k)=mod(hat_r_ic(3*i-2,k)+delta_r(3*i-2,k),2*pi);%方向估计
        Eigen::Matrix2d R_d_phi;
        Eigen::Matrix2d B_d_phi;
        R_d_phi << cos(delta_r(0)), -sin(delta_r(0)),
            sin(delta_r(0)), cos(delta_r(0));
        B_d_phi << sin(delta_r(0)), cos(delta_r(0)) - 1,
            1 - cos(delta_r(0)), sin(delta_r(0));
        hat_r1.block<2, 1>(1, 0) = R_d_phi * hat_r_ic.block<2, 1>(1, 0) + 1 / delta_r(0) * B_d_phi * delta_r.block<2, 1>(1, 0);
        // hat_r1(3*i-1:3*i,k)=[cos(delta_r(3*i-2,k)),-sin(delta_r(3*i-2,k));sin(delta_r(3*i-2,k)),cos(delta_r(3*i-2,k))]*hat_r_ic(3*i-1:3*i,k)+inv(delta_r(3*i-2,k))*[sin(delta_r(3*i-2,k)),cos(delta_r(3*i-2,k))-1;1-cos(delta_r(3*i-2,k)),sin(delta_r(3*i-2,k))]*delta_r(3*i-1:3*i,k);%位置估计
    }
}

void SensorNode::propagation_step(int time_k)
{
    hat_r0(0) = Utils::periodical_clamp(hat_r1(0) + vel_leader(0) * dt, -PI, PI, 2*PI);
    // hat_r0(3*i-2,k+1)=mod(hat_r1(3*i-2,k)+omega,2*pi);%下一时刻方向的先验估计
    Eigen::Matrix2d hat_R;
    hat_R << cos(hat_r1(0)), -sin(hat_r1(0)),
        sin(hat_r1(0)), cos(hat_r1(0));
    // hat_R=[cos(hat_r1(3*i-2,k)),-sin(hat_r1(3*i-2,k));sin(hat_r1(3*i-2,k)),cos(hat_r1(3*i-2,k))];
    hat_r0.block<2, 1>(1, 0) = hat_r1.block<2, 1>(1, 0) + hat_R * vel_leader.block<2, 1>(1, 0) * dt;
    // hat_r0(3*i-1:3*i,k+1)=hat_r1(3*i-1:3*i,k)+hat_R*v;%下一时刻位置的先验估计
    Eigen::Matrix2d J;
    J << 0, -1,
        1, 0;
    Eigen::Vector2d b = - J * (hat_r1.block<2, 1>(1, 0) + hat_R * vel_leader.block<2, 1>(1, 0) * dt);
    // b=-J*(hat_r1(3*i-1:3*i,k)+hat_R*v);
    Eigen::Matrix3d B;
    B << 1, 0, 0,
        b, hat_R;
    //B(3*i-2:3*i,3*k-2:3*k)=[1,0,0;b,hat_R];
    P_0 = P_1 + B * Q * B.transpose();
    // P_0(3*i-2:3*i,3*k+1:3*k+3)=P_1(3*i-2:3*i,3*k-2:3*k)+B(3*i-2:3*i,3*k-2:3*k)*Q*B(3*i-2:3*i,3*k-2:3*k)';%协方差的先验估计

    std::cout << index << ":" << hat_r1.transpose() << std::endl;
    if(pose_history.size > 0)
    {
        if((pose_history.get_data(0).block<2, 1>(1, 0) - hat_r1.block<2, 1>(1, 0)).norm() > 0.02)
        {
            pose_history.append(hat_r1);
        }
    }
    else
    {
        pose_history.append(hat_r1);
    }

    if(pose_history.size > 25 * (index + 1))
    {
        pos_follow = pose_history.get_data(25 * (index + 1)).block<2, 1>(1, 0);
    }
    // pos_follow = hat_r1.block<2, 1>(1, 0);
}

EstimationSystem::EstimationSystem()
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
    
    time_k = 0;
    alpha_0 = 0.2;
    tau_alpha = 0.75;
    for(int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].index = i;
    }

}

double EstimationSystem::get_alpha_k()
{
    return alpha_0 / pow(time_k + 1, tau_alpha);
}

void EstimationSystem::update_leader_pose_vel(Eigen::Vector3d pos, Eigen::Vector3d vel)
{
    pos_leader = pos;
    vel_leader = vel;
    for(int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].pos_leader = pos;
        sensors[i].vel_leader = vel;
    }
}

void EstimationSystem::estimation_step(Eigen::Vector3d pos_leader, Eigen::Vector3d vel)
{
    if(pos_leader.norm() + vel.norm() > 200)
    {
        return;
    }
    update_leader_pose_vel(pos_leader, vel);
    consensus_step();
    update_step();
    propagation_step();
    time_k ++;
}

void EstimationSystem::consensus_step()
{
    Eigen::Matrix<double, NUM_OF_SENSOR, NUM_OF_SENSOR> A;
    A = time_k % 2 ? A2 : A1;
    Eigen::Matrix<double, NUM_OF_SENSOR * 3, 1> hat_r0_all;
    for(int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        hat_r0_all.block<3, 1>(i * 3, 0) = sensors[i].hat_r0;
    }
    for(int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].hat_r_ic = (1 - get_alpha_k() * (A.block<1, NUM_OF_SENSOR>(i, 0) * Eigen::Matrix<double, 5, 1>::Ones() - A(i, i))) * sensors[i].hat_r0 +
                                get_alpha_k() * (Eigen::kroneckerProduct(A.block<1, NUM_OF_SENSOR>(i, 0), Eigen::Matrix3d::Identity()) * hat_r0_all - A(i, i) * sensors[i].hat_r0);
        //hat_r_ic(3*i-2:3*i,k)=(1-alpha(k)*(A(i,:)*ones(5,1)-A(i,i)))*hat_r0(3*i-2:3*i,k)+alpha(k)*(kron(A(i,:),eye(3))*hat_r0(:,k)-A(i,i)*hat_r0(3*i-2:3*i,k));
        Eigen::Matrix3d tilde_P = Eigen::Matrix3d::Zero();
        for (int j = 0; j < NUM_OF_SENSOR; ++j)
        {
            tilde_P = tilde_P + A(i, j) * sensors[j].P_0.inverse();
            //tilde_P=tilde_P+A(i,j)*inv(P_0(3*j-2:3*j,3*k-2:3*k));
        }
        sensors[i].P_ic = ((1 - get_alpha_k() * (A.block<1, NUM_OF_SENSOR>(i, 0) * Eigen::Matrix<double, 5, 1>::Ones() - A(i, i))) * sensors[i].P_0.inverse() + 
                            get_alpha_k() * (tilde_P - A(i, i) * sensors[i].P_0.inverse())).inverse();
        //P_ic(3*i-2:3*i,3*k-2:3*k)=inv((1-alpha(k)*(A(i,:)*ones(5,1)-A(i,i)))*inv(P_0(3*i-2:3*i,3*k-2:3*k))+alpha(k)*(tilde_P-A(i,i)*inv(P_0(3*i-2:3*i,3*k-2:3*k))));%融合后的协方差
        
    }
}

void EstimationSystem::update_step()
{
    for(int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].update_step(time_k);
    }
}

void EstimationSystem::propagation_step()
{
    for(int i = 0; i < NUM_OF_SENSOR; ++i)
    {
        sensors[i].propagation_step(time_k);
    }
}
