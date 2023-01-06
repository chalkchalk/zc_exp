#include "follower.h"
#include "utils.h"

Follower::Follower(ros::NodeHandle &nh, std::string name, Eigen::Vector2d max_velocity, Eigen::Vector2d follow_kp) : nh_(nh), name_(name), max_velocity_(max_velocity), follow_kp_(follow_kp)
{
    pose_received_ = false;
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(name_ + "/cmd_vel", 100);
    pub_follow_pose = nh.advertise<geometry_msgs::PointStamped>(name_ + "/follow_pose", 100);
    bool is_simulation;
    nh.getParam("is_simulation", is_simulation);
    current_pose_ = Eigen::Vector3d(1e5, 1e5, 1e5);
    if(is_simulation)
    {
        std::cout << "use_simulation_pose:" << std::endl;
        sub_current_pose_ = nh.subscribe(name_ + "/ground_truth_odom", 1, &Follower::gazebo_real_pose_callback, this);
    }
    else
    {
        
    }
}

void Follower::gazebo_real_pose_callback(const nav_msgs::Odometry &pose)
{
    pose_received_ = true;
    current_pose_(1) = pose.pose.pose.position.x;
    current_pose_(2) = pose.pose.pose.position.y;
    current_pose_(0) = Utils::quat_to_euler(Eigen::Quaterniond(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z))(2);
}

void Follower::calculate_follow_speed()
{
    Eigen::Vector2d pose_now = Eigen::Vector2d(current_pose_(1), current_pose_(2));
    // std::cout << name_ + ": pose_now = " << pose_now.transpose() << std::endl;
    Eigen::Vector2d goal_delta_pose = following_target_ - pose_now;
    double goal_angle = atan2(goal_delta_pose(1), goal_delta_pose(0));
    // std::cout << name_ + ": goal_angle = " << goal_angle << std::endl;
    // std::cout << name_ + ": goal_delta_pose = " << goal_delta_pose.transpose() << std::endl;
    double delta_angle = Utils::periodical_clamp(goal_angle - current_pose_(0), -PI, PI, 2 * PI);
    cmd_vel_.linear.x = Utils::clamp(follow_kp_(0) * goal_delta_pose.norm() * cos(delta_angle), -max_velocity_(0),  max_velocity_(0));
    cmd_vel_.angular.z = Utils::clamp(follow_kp_(1) * delta_angle, -max_velocity_(1),  max_velocity_(1));
    if(goal_delta_pose.norm() < 0.01 || goal_delta_pose.norm() > 100)
    {
        cmd_vel_.linear.x = 0;
        cmd_vel_.angular.z = 0;
    }
}

void Follower::follow(Eigen::Vector2d target)
{
    following_target_ = target;
    if(!pose_received_)
    {
        return;
    }
    calculate_follow_speed();

    // cmd_vel_.linear.x = 0;
    // cmd_vel_.angular.z = 0;


    pub_cmd_vel.publish(cmd_vel_);
    publish_follow_pose();
}

void Follower::publish_tf()
{
    tf::Transform transform_base;
    transform_base.setOrigin(tf::Vector3(current_pose_(1), current_pose_(2), 0.009));
    Eigen::Quaterniond q = Utils::euler_to_quat(Eigen::Vector3d(0, 0, current_pose_(0)));
    tf::Quaternion q_base(q.x(), q.y(), q.z(), q.w());
    transform_base.setRotation(q_base);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_base, ros::Time::now(), "world", name_ + "/base_footprint"));
}

void Follower::publish_follow_pose()
{
    geometry_msgs::PointStamped pose_follow;
    pose_follow.header.frame_id = "world";
    pose_follow.header.stamp = ros::Time::now();
    pose_follow.point.x = following_target_(0);
    pose_follow.point.y = following_target_(1);
    pose_follow.point.z = 0;
    pub_follow_pose.publish(pose_follow);
}