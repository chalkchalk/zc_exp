#include "experiment_node.h"
#include "utils.h"

ExpLog::ExpLog()
{
    fout.open("/root/catkin_ws/src/zc_exp/src/log/" + Utils::get_date_time() +".csv", std::ios::out | std::ios::app);
    std::cout << "log file created" << std::endl;
    fout << "leader_x" << "," << "leader_y" << "," << "leader_theta" << ",";
    fout << "leader_vel_linear" << "," << "leader_vel_angular";
    for(int i = 0 ;i < NUM_OF_SENSOR; i++)
    {
        std::string sensor_name = "sensor_" + std::to_string(i);
        fout << sensor_name + "_x" << "," << sensor_name + "_y" << "," << sensor_name + "_theta" << ",";
        fout << sensor_name + "_vel_linear" << "," << sensor_name + "_vel_angular" << ",";
        fout << "estimated_x" << "," << "estimated_y" << "," << "estimated_theta";
    }
    fout << "\n";

}

void ExpLog::record(double data[8 * NUM_OF_SENSOR + 5])
{
    
}

Experiment::Experiment(ros::NodeHandle &nh) : nh_(nh)
{
    leader = new Leader(nh_,"bot1");
    follower[0] = new Follower(nh_,"bot2");
    follower[1] = new Follower(nh_,"bot3");
    follower[2] = new Follower(nh_,"bot4");
    follower[3] = new Follower(nh_,"bot5");
    follower[4] = new Follower(nh_,"bot6");
}

void Experiment::publish_tf()
{
    for(int i = 0 ;i < NUM_OF_SENSOR; i++)
    {
        follower[i]->publish_tf();
    }
    leader->publish_tf();
}

void Experiment::step()
{
    leader->move();
    for(int i = 0 ;i < NUM_OF_SENSOR; i++)
    {
        estimation_system.sensors[i].pos_now = follower[i]->get_current_pose();
        follower[i]->follow(estimation_system.sensors[i].pos_follow);
        //follower[i]->follow(Eigen::Vector2d(i + 1, - i * 2 - 1));
    }
    estimation_system.estimation_step(leader->get_current_pose(), leader->get_current_vel());

    publish_tf();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiment_node");
    ros::NodeHandle nh;
    Experiment experiment(nh);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::spinOnce();
        experiment.step();
        ros::Duration dt_solver_time = ros::Time::now() - now;
        if (dt_solver_time.toSec() < INTERVAL_MS / 1000.0) 
        {
            ros::Duration( INTERVAL_MS / 1000.0 - dt_solver_time.toSec() ).sleep();
        }
    }
    return 0;
}