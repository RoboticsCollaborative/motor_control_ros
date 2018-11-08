//
// Created by ur5 on 11/6/18.
//

#include "../../include/nodes/rdd_node.h"
#include <pthread.h>

using namespace std;

RDDNode::RDDNode(ros::NodeHandle &node) {
    nh_ = node;
    set_torque_sub = nh_.subscribe("/rdd/set_torque", 1, &RDDNode::set_torque_callback, this);
    actual_position_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_position", 1);
    actual_velocity_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_velocity", 1);
}

RDDNode::~RDDNode() {}

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
        position_msg.data = 1.1;
//        position_msg.data = getActualPosition(1);
        actual_position_pub.publish(position_msg);
        cout<<"published"<<endl;

        std_msgs::Float64 velocity_msg;
//        velocity_msg.data = getActualVelocity(1);
        velocity_msg.data = 2.1;
        actual_velocity_pub.publish(velocity_msg);

        ros::spinOnce();
        rate.sleep();
    }
}

//boost::thread* RDDNode::start_ros(int argc, char **argv)
//{
//    ros::init(argc, argv, "rdd");
//    ros::NodeHandle node("~");
//    RDDNode* rdd = new RDDNode(node);
//
//    boost::thread* ros_thread = new boost::thread(&RDDNode::run, rdd);
//    return ros_thread;
//}

void* ros_loop(void */*unused_param*/)
{
    ros::NodeHandle node("~");
    RDDNode rdd(node);
    rdd.run();
    return NULL;
}

static pthread_t ros_thread;
static pthread_attr_t ros_thread_attr;

int main(int argc, char** argv)
{
//    boost::thread* ros_thread = RDDNode::start_ros(argc, argv);
    ros::init(argc, argv, "rdd");
    int rv = pthread_create(&ros_thread, &ros_thread_attr, ros_loop, NULL);
    if (rv != 0)
    {
        ROS_FATAL("Unable to create control thread: rv = %d", rv);
        exit(EXIT_FAILURE);
    }
    ros::spin();
    pthread_join(ros_thread, reinterpret_cast<void **>(&rv));
    return rv;
}