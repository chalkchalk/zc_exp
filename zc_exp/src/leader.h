#ifndef LEADER_H_
#define LEADER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"
#include <tf/transform_broadcaster.h>

class Leader
{
public:
    Leader(ros::NodeHandle &nh, std::string name, Eigen::Vector2d max_velocity=Eigen::Vector2d(0.15, 1.0));
    Eigen::Vector3d get_current_pose(){return current_pose_;}
    Eigen::Vector3d get_current_vel(){return velocity_now_;}
    void move();
    void publish_tf();
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_vel;
    ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_joy_;
    std::string name_;
    Eigen::Vector2d max_velocity_; 
    Eigen::Vector3d velocity_now_; // theta, x, y,
    geometry_msgs::Twist cmd_vel_;
    Eigen::Vector3d current_pose_; // theta, x, y,
    tf::TransformBroadcaster tf_broadcaster_;

    void gazebo_real_pose_callback(const nav_msgs::Odometry &pose);
    void vrpn_pose_callback(const geometry_msgs::PoseStamped &pose);
    void gazebo_odom_callback(const nav_msgs::Odometry &odom);
    void joy_msg_callback(const sensor_msgs::Joy &msg);
};


#endif
