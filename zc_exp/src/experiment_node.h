#ifndef EXPERIMENT_NODE_H_
#define EXPERIMENT_NODE_H_

#include <ros/ros.h>
#include "follower.h"
#include "leader.h"
#include "estimation.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#define DATA_SENSOR 6
#define DATA_LEADER 5
#define LOG_LEN (NUM_OF_SENSOR * DATA_SENSOR + DATA_LEADER)
class ExpLog
{
public:
    ExpLog();
    void record(double data[LOG_LEN]);   
private:
    std::fstream fout;
};


class Experiment
{
public: 
    Experiment(ros::NodeHandle &nh);
    void step();

private:
    ros::NodeHandle nh_;
    EstimationSystem estimation_system;
    Follower *follower[NUM_OF_SENSOR];
    Leader *leader;
    ExpLog log;
    void publish_tf();
    void record_log();
};

#endif