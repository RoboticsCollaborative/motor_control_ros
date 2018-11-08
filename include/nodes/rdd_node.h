//
// Created by ur5 on 11/6/18.
//

#ifndef MOTOR_CONTROL_ROS_RDD_NODE_H
#define MOTOR_CONTROL_ROS_RDD_NODE_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "../motor_control_ros/ros_interface.h"
#include <boost/thread/thread.hpp>


class RDDNode
{
public:
    explicit RDDNode(ros::NodeHandle& node);

    ~RDDNode();

    void run();

//    static boost::thread* start_ros(int argc, char **argv);

private:
    void set_torque_callback(const std_msgs::Int16& msg);

    ros::NodeHandle nh_;
    ros::Subscriber set_torque_sub;
    ros::Publisher actual_position_pub;
    ros::Publisher actual_velocity_pub;

};


#endif //MOTOR_CONTROL_ROS_RDD_NODE_H
