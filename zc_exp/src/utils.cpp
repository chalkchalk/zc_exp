#include "utils.h"


Eigen::Quaterniond Utils::euler_to_quat(Eigen::Vector3d rpy)
{
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    //Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Quaterniond q =  yawAngle * pitchAngle * rollAngle;
    return q;
}


Eigen::Matrix3d Utils::euler_to_angular_velocity_mapping(Eigen::Vector3d rpy)
{
    Eigen::Matrix3d mapping;
    mapping << cos(rpy(1)) * cos(rpy(2)), -sin(rpy(2)), 0,
               cos(rpy(1)) * sin(rpy(2)), cos(rpy(2)), 0,
               -sin(rpy(1)), 0, 1;
    return mapping;
}


std::vector<Eigen::Triplet<double>> Utils::matrix_to_none_zero_tripletList(Eigen::MatrixXd matrix, int start_i, int start_j, double eps)
{
    std::vector<Eigen::Triplet<double>> triplet_lists;
    for(int i = 0; i < matrix.rows(); i++)
    {
        for(int j = 0; j < matrix.cols(); j++)
        {
            if(abs(matrix(i, j)) > eps)
            {
                triplet_lists.emplace_back(Eigen::Triplet<double>(i + start_i, j + start_j, matrix(i, j)));
            }
        }
    }
    return triplet_lists;
}

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}
//seems to be the cross product of matrix
Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}


double Utils::periodical_clamp(double value, double lower_bound, double upper_bound, double T)
{
    assert(("T must be positive!", T > 0.0));
    assert(("upper_bound - lower_bound must >= T!", upper_bound - lower_bound >= T - 1e-4));
    while(value > upper_bound)
    {
        value -= T;
    }
    while(value < lower_bound)
    {
        value += T;
    }
    return value;
}

double Utils::clamp(double value, double lower_bound, double upper_bound)
{
    assert(("the up bound must be larger than or equal to the lower bound!", upper_bound >= lower_bound));
    return std::max(lower_bound, std::min(upper_bound, value));
}

Eigen::Vector3d Utils::clamp(Eigen::Vector3d value, Eigen::Vector3d lower_bound, Eigen::Vector3d upper_bound)
{
    return Eigen::Vector3d(std::max(lower_bound(0), std::min(upper_bound(0), value(0))),
                           std::max(lower_bound(1), std::min(upper_bound(1), value(1))),
                           std::max(lower_bound(2), std::min(upper_bound(2), value(2))));
}
Eigen::Vector3d Utils::frame_transformation(Eigen::Vector3d src_vector, Eigen::Matrix3d rotation, Eigen::Vector3d transition)
{
    return rotation * src_vector + transition;
}


Eigen::Quaterniond Utils::xyz_pos_to_quaternion(Eigen::Vector3d pos)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix;
    if(pos.isZero())
    {
        rotation_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    }
    else
    {
        //std::cout<<"pos:"<<std::endl;
        //std::cout<<pos<<std::endl;
        Eigen::Vector3d pos_normalized = pos.normalized();
        //std::cout<<"pos_normalized:"<<std::endl;
        //std::cout<<pos_normalized<<std::endl;
        Eigen::Vector3d pos_normalized_per;
        if(abs(pos_normalized(0)) + abs(pos_normalized(1)) > 0)
        {
            pos_normalized_per << -pos_normalized(1), pos_normalized(0), 0;
        }
        else
        {
            pos_normalized_per << 0, -pos_normalized(2), pos_normalized(1);
        }
        pos_normalized_per = pos_normalized_per.normalized(); 
        //std::cout<<"pos_normalized_per:"<<std::endl;
        //std::cout<<pos_normalized_per<<std::endl;
        Eigen::Vector3d pos_normalized_per2 = pos_normalized.cross(pos_normalized_per);
        //std::cout<<"pos_normalized_per2:"<<std::endl;
        //std::cout<<pos_normalized_per2<<std::endl;
        rotation_matrix << pos_normalized, pos_normalized_per, pos_normalized_per2;
        //std::cout<<"rotation_matrix:"<<std::endl;
        //std::cout<<rotation_matrix<<std::endl;
        //rotation_matrix.transposeInPlace();
    }
    //std::cout<<"rotation_matrix:"<<std::endl;
    //std::cout<<rotation_matrix<<std::endl;
    Eigen::Quaterniond quaternion(rotation_matrix);
    return quaternion;
}

Eigen::Vector2d Utils::calculate_displacement(Eigen::Vector2d lin_vel, double yaw_rate, double dt, bool use_approximated)
{
    Eigen::Vector2d displacement;
    if(yaw_rate != 0)
    {
        displacement(0) = 1.0 / yaw_rate * (lin_vel(0) * sin(yaw_rate * dt) - lin_vel(1) * (1 - cos(yaw_rate * dt)));
        displacement(1) = 1.0 / yaw_rate * (lin_vel(0) * (1 - cos(yaw_rate * dt)) + lin_vel(1) * sin(yaw_rate * dt));
    }
    else
    {
        displacement = lin_vel * dt;
    }
    return displacement;
}



double Utils::calculate_included_angle(Eigen::Vector3d vec1, Eigen::Vector3d vec2)
{
    if((vec1.transpose() * vec2)(0) == 0)
    {
        return 0.5 * PI;
    }
    else
    {
        return acos((vec1.transpose() * vec2)(0) / (vec1.norm() * vec2.norm()));
    }
}

Eigen::Matrix2d Utils::yaw_to_rotaion_mat(double yaw)
{
    Eigen::Matrix2d mat;
    mat << cos(yaw), -sin(yaw),
        sin(yaw), cos(yaw);
    return mat;
}
