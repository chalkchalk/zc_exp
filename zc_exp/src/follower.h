#ifndef FOLLOWER_H_
#define FOLLOWER_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


class Follower
{
public:
    Follower(ros::NodeHandle &nh, std::string name, Eigen::Vector2d max_velocity=Eigen::Vector2d(0.2, 2.8), Eigen::Vector2d follow_kp=Eigen::Vector2d(0.8, 0.8));
    void follow(Eigen::Vector2d target);
    Eigen::Vector3d get_current_pose(){return current_pose_;}
    void publish_tf();
    void stop();
private: 
    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_follow_pose;
    ros::Subscriber sub_current_pose_;
    std::string name_;
    Eigen::Vector2d max_velocity_;
    Eigen::Vector2d follow_kp_;
    Eigen::Vector2d following_target_;
    geometry_msgs::Twist cmd_vel_;
    Eigen::Vector3d current_pose_; // theta, x, y,
    bool pose_received_;
    tf::TransformBroadcaster tf_broadcaster_;

    void gazebo_real_pose_callback(const nav_msgs::Odometry &pose);
    void vrpn_pose_callback(const geometry_msgs::PoseStamped &pose);
    void calculate_follow_speed();
    void publish_follow_pose();
};

#endif