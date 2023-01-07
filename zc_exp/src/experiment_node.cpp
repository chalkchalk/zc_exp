#include "experiment_node.h"
#include "utils.h"

ExpLog::ExpLog()
{
    fout.open("/root/catkin_ws/src/zc_exp/src/log/" + Utils::get_date_time() +".csv", std::ios::out | std::ios::app);
    std::cout << "log file created" << std::endl;
    fout << "leader_x" << "," << "leader_y" << "," << "leader_theta" << ",";
    fout << "leader_vel_linear" << "," << "leader_vel_angular" << ",";
    for(int i = 0 ;i < NUM_OF_SENSOR; i++)
    {
        std::string sensor_name = "sensor_" + std::to_string(i + 1);
        fout << sensor_name + "_x" << "," << sensor_name + "_y" << "," << sensor_name + "_theta" << ",";
        fout << "estimated_x" << "," << "estimated_y" << "," << "estimated_theta";
        if(i < NUM_OF_SENSOR - 1)
        {
            fout << ",";
        }

    }
    fout << "\n";

}

void ExpLog::record(double data[LOG_LEN])
{
    for(int i = 0 ;i < LOG_LEN; i++)
    {
        fout << data[i];
        if(i < LOG_LEN - 1)
        {
            fout << ",";
        }
    }
    fout << "\n";
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
    record_log();
    publish_tf();
}

void Experiment::record_log()
{
    double data[LOG_LEN];
    data[0] = leader->get_current_pose()(1);
    data[1] = leader->get_current_pose()(2);
    data[2] = leader->get_current_pose()(0);
    data[3] = leader->get_current_vel()(1);
    data[4] = leader->get_current_vel()(0);
    for(int i = 0 ;i < NUM_OF_SENSOR; i++)
    {
        data[i * DATA_SENSOR + DATA_LEADER + 0] = follower[i]->get_current_pose()(1);
        data[i * DATA_SENSOR + DATA_LEADER + 1] = follower[i]->get_current_pose()(2);
        data[i * DATA_SENSOR + DATA_LEADER + 2] = follower[i]->get_current_pose()(0);
        data[i * DATA_SENSOR + DATA_LEADER + 3] = estimation_system.sensors[i].hat_r1(1);
        data[i * DATA_SENSOR + DATA_LEADER + 4] = estimation_system.sensors[i].hat_r1(2);
        data[i * DATA_SENSOR + DATA_LEADER + 5] = estimation_system.sensors[i].hat_r1(0);
    }
    log.record(data);
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


