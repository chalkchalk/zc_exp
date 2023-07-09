#include "leader.h"
#include "utils.h"

Leader::Leader(ros::NodeHandle &nh, std::string name, Eigen::Vector2d max_velocity) : nh_(nh), name_(name), max_velocity_(max_velocity)
{
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(name_ + "/cmd_vel", 100);
    event_received_ = false;
    nh.getParam("is_simulation", is_simulation);
    if(is_simulation)
    {
        sub_odom_ = nh.subscribe(name_ + "/odom", 1, &Leader::odom_callback, this);
        sub_current_pose_ = nh.subscribe(name_ + "/ground_truth_odom", 1, &Leader::gazebo_real_pose_callback, this);
    }
    else
    {
        std::cout << "use_vrpn_pose:" << std::endl;
        sub_odom_ = nh.subscribe(name_ + "/pose", 1, &Leader::odom_callback, this);
        sub_current_pose_ = nh.subscribe("/vrpn_client_node/" + name_ + "/pose", 1, &Leader::vrpn_pose_callback, this);
    }
    sub_joy_ = nh_.subscribe("/joy", 10, &Leader::joy_msg_callback, this); 
    current_pose_ = Eigen::Vector3d(1e5, 1e5, 1e5);
    velocity_now_ = Eigen::Vector3d(1e5, 1e5, 1e5);
    vel_received = false;
}

void Leader::vrpn_pose_callback(const geometry_msgs::PoseStamped &pose)
{
    current_pose_(1) = pose.pose.position.x;
    current_pose_(2) = pose.pose.position.y;
    current_pose_(0) = Utils::quat_to_euler(Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z))(2);
}

void Leader::gazebo_real_pose_callback(const nav_msgs::Odometry &pose)
{
    current_pose_(1) = pose.pose.pose.position.x;
    current_pose_(2) = pose.pose.pose.position.y;
    current_pose_(0) = Utils::quat_to_euler(Eigen::Quaterniond(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z))(2);
}

void Leader::odom_callback(const nav_msgs::Odometry &odom)
{
    velocity_now_ = Eigen::Vector3d(odom.twist.twist.angular.z, odom.twist.twist.linear.x, odom.twist.twist.linear.y);
}

void Leader::joy_msg_callback(const sensor_msgs::Joy &msg)
{
    cmd_vel_.linear.x = msg.axes[1] * max_velocity_(0);
    cmd_vel_.angular.z = msg.axes[2] * max_velocity_(1);

    if(cmd_vel_.linear.x > 0.05)
    {
        vel_received = true;
    }

    if(msg.buttons[0])
    {
        event_received_ = true;
        std::cout << "event_received: true" << std::endl;
    }
    if(msg.buttons[1])
    {
        event_received_ = false;
        std::cout << "event_received: false" << std::endl;
    }

}

void Leader::move()
{
    pub_cmd_vel.publish(cmd_vel_);
}

void Leader::publish_tf()
{
    tf::Transform transform_base;
    transform_base.setOrigin(tf::Vector3(current_pose_(1), current_pose_(2), 0.009));
    Eigen::Quaterniond q = Utils::euler_to_quat(Eigen::Vector3d(0, 0, current_pose_(0)));
    tf::Quaternion q_base(q.x(), q.y(), q.z(), q.w());
    transform_base.setRotation(q_base);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_base, ros::Time::now(), "world", name_ + "/base_footprint"));
}