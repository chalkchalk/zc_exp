#ifndef LEADER_H_
#define LEADER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class Leader
{
public:
    Leader(ros::NodeHandle &nh, std::string name, Eigen::Vector2d max_velocity=Eigen::Vector2d(0.15, 1.0));
    Eigen::Vector3d get_current_pose(){return current_pose_;}
    Eigen::Vector3d get_current_vel(){return velocity_now_;}
    void move();
    void publish_tf();
    bool has_moved(){return vel_received;}
    bool event_received(){return event_received_;}
    bool stop_all(){return stop_all_;}
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_vel;
    ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_joy_;
    std::string name_;
    bool is_simulation;
    bool event_received_;
    bool stop_all_;
    Eigen::Vector2d max_velocity_; 
    Eigen::Vector3d velocity_now_; // theta, x, y,
    geometry_msgs::Twist cmd_vel_;
    Eigen::Vector3d current_pose_; // theta, x, y,
    tf::TransformBroadcaster tf_broadcaster_;
    bool vel_received;
    void gazebo_real_pose_callback(const nav_msgs::Odometry &pose);
    void vrpn_pose_callback(const geometry_msgs::PoseStamped &pose);
    void odom_callback(const nav_msgs::Odometry &odom);
    void joy_msg_callback(const sensor_msgs::Joy &msg);
};


#endif
