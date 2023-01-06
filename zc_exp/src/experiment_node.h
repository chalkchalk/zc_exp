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

class ExpLog
{
public:
    ExpLog();
    void record(double data[8 * NUM_OF_SENSOR + 5]);   
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
};

#endif