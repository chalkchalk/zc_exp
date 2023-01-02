//
// Created by shuoy on 10/19/21.
//

#ifndef A1_CPP_UTILS_H
#define A1_CPP_UTILS_H

#define COUT_RESET   "\033[0m"
#define COUT_BLACK   "\033[30m"      /* Black */
#define COUT_RED     "\033[31m"      /* Red */
#define COUT_GREEN   "\033[32m"      /* Green */
#define COUT_YELLOW  "\033[33m"      /* Yellow */
#define COUT_BLUE    "\033[34m"      /* Blue */

#define PI 3.141592653589793115997963468544185161590576171875
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>


#define SIGN(x) (((x) > 0) - ((x) < 0))


#define QUEUE_LEN 500

template<class T> class Queue
{
public:
    Queue()
    {
        counter = 0;
        size = 0;
    };
    int counter;
    int size;
    T data[QUEUE_LEN];

    void append(T new_data)
    {
        data[counter] = new_data;
        counter ++;
        size ++;
        size = std::min(size, QUEUE_LEN);
        if(counter == QUEUE_LEN)
        {
            counter = 0;
        }
    }

    T get_data(int t)
    {
        if(t >= QUEUE_LEN)
        {
            std::cout << "invalid input!" << std::endl;
        }
        if(counter - t - 1 >= 0)
        {
            return data[counter - t - 1];
        }
        else
        {
            return data[counter - t - 1 + QUEUE_LEN];
        }
    }

};


class Utils {
public:
    // compare to Eigen's default eulerAngles
    
    static Eigen::Quaterniond euler_to_quat(Eigen::Vector3d rpy);
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);// this function returns yaw angle within -pi to pi
    static Eigen::Matrix3d euler_to_angular_velocity_mapping(Eigen::Vector3d rpy);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double periodical_clamp(double value, double lower_bound, double upper_bound, double T);
    static double clamp(double value, double lower_bound, double upper_bound);
    static Eigen::Vector3d clamp(Eigen::Vector3d value, Eigen::Vector3d lower_bound, Eigen::Vector3d upper_bound);
    static Eigen::Quaterniond xyz_pos_to_quaternion(Eigen::Vector3d pos);
    static Eigen::Vector3d frame_transformation(Eigen::Vector3d src_vector, Eigen::Matrix3d rotation, Eigen::Vector3d transition);
    // template <typename T> 
    static std::vector<Eigen::Triplet<double>> matrix_to_none_zero_tripletList(Eigen::MatrixXd matrix, int start_i=0, int start_j=0, double eps=1e-7);
    static Eigen::Vector2d calculate_displacement(Eigen::Vector2d lin_vel, double yaw_rate, double dt, bool use_approximated=false);
    static double calculate_included_angle(Eigen::Vector3d vec1, Eigen::Vector3d vec2);
    static Eigen::Matrix2d yaw_to_rotaion_mat(double yaw);
};




#endif 
