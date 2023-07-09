#ifndef EXPERIMENT_NODE_H_
#define EXPERIMENT_NODE_H_


#define COMPARE

#include <ros/ros.h>
#include "follower.h"
#include "leader.h"
#ifdef COMPARE
#include "estimation_compare.h"
#else
#include "estimation.h"
#endif

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
    bool is_start;
    bool is_simulation;
    ros::NodeHandle nh_;

    #ifdef COMPARE
    EstimationSystemCompare estimation_system;
    #else
    EstimationSystem estimation_system;
    #endif
    Follower *follower[NUM_OF_SENSOR];
    Leader *leader;
    ExpLog log;
    void publish_tf();
    void record_log();
};

#endif