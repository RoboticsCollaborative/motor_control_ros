//
// Created by ur5 on 11/6/18.
//

#include "../../include/nodes/rdd_node.h"
#include "../../include/motor_control_ros/ros_interface.h"

RDDNode::RDDNode(ros::NodeHandle &node) {
    nh_ = node;
    set_torque_sub = nh_.subscribe("/rdd/set_torque", 1, &RDDNode::set_torque_callback, this);
    actual_position_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_position", 1);
    actual_velocity_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_velocity", 1);
}

RDDNode::~RDDNode() {

}

void RDDNode::set_torque_callback(const std_msgs::Int16& msg)
{
    setTargetTorque(1, msg.data);
}

void RDDNode::run()
{
    ros::Rate rate(100);
    while (ros::ok())
    {
        std_msgs::Float64 position_msg;
        position_msg.data = getActualPosition(1);
        actual_position_pub.publish(position_msg);

        std_msgs::Float64 velocity_msg;
        velocity_msg.data = getActualVelocity(1);
        actual_velocity_pub.publish(velocity_msg);
    }
}