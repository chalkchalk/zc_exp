#ifndef EXPERIMENT_NODE_H_
#define EXPERIMENT_NODE_H_

#include <ros/ros.h>
#include "follower.h"
#include "leader.h"
#include "estimation.h"


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
    void publish_tf();
};

#endif