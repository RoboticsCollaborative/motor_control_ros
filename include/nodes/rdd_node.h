//
// Created by ur5 on 11/6/18.
//

#ifndef MOTOR_CONTROL_ROS_RDD_NODE_H
#define MOTOR_CONTROL_ROS_RDD_NODE_H

// c++
#include <pthread.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>

//self
#include "../motor_control_ros/ros_interface.h"
extern "C" {
#include "../motor_control_ros/haptic_config.h"
};

static pthread_t ros_thread;
static pthread_attr_t ros_thread_attr;

class RDDNode
{
public:
    explicit RDDNode(ros::NodeHandle& node);

    ~RDDNode();

    void run();

    static int start_ros_thread();
    static int join_ros_thread(int rv);

private:
    static void* ros_loop(void* node_ptr);

    void set_torque_callback(const std_msgs::Int16ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber set_torque_sub;
    ros::Publisher actual_position_pub;
    ros::Publisher actual_velocity_pub;

};


#endif //MOTOR_CONTROL_ROS_RDD_NODE_H
