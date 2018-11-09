//
// Created by ur5 on 11/6/18.
//

#include "../../include/nodes/rdd_node.h"

using namespace std;

RDDNode::RDDNode(ros::NodeHandle &node) {
    nh_ = node;
    set_torque_sub = nh_.subscribe("/rdd/set_torque", 1, &RDDNode::set_torque_callback, this);
    actual_position_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_position", 1);
    actual_velocity_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_velocity", 1);
}

RDDNode::~RDDNode() {}

void RDDNode::set_torque_callback(const std_msgs::Int16ConstPtr& msg)
{
//    setTargetTorque(1, msg.data);
    ROS_INFO("get torque: %d", msg->data);
}

void RDDNode::run()
{
    ros::Rate rate(2);
    while (ros::ok())
    {
        if (not inOP){
            ROS_INFO("driver not ready");
            rate.sleep();
            continue;
        }
        ROS_INFO("driver ready");
        std_msgs::Float64 position_msg;
//        position_msg.data = 1.1;
        position_msg.data = getActualPosition(1);
	ROS_INFO("get position");
        actual_position_pub.publish(position_msg);

        std_msgs::Float64 velocity_msg;
//        velocity_msg.data = 2.1;
        velocity_msg.data = getActualVelocity(1);
	ROS_INFO("get velocity");
        actual_velocity_pub.publish(velocity_msg);

        ros::spinOnce();
        rate.sleep();
    }
}

void* RDDNode::ros_loop(void *node_ptr)
{
    RDDNode* rdd = (RDDNode*) node_ptr;
    rdd->run();
    delete(rdd);
    return NULL;
}

int RDDNode::start_ros_thread()
{
    ros::NodeHandle node("~");
    RDDNode* rdd = new RDDNode(node);
    int rv = pthread_create(&ros_thread, &ros_thread_attr, RDDNode::ros_loop, rdd);
    if (rv != 0)
    {
        ROS_FATAL("Unable to create control thread: rv = %d", rv);
        exit(EXIT_FAILURE);
    }
    return rv;
}

int RDDNode::join_ros_thread(int rv)
{
    pthread_join(ros_thread, reinterpret_cast<void **>(&rv));
    return rv;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rdd");
    int rv = RDDNode::start_ros_thread();

    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        osal_thread_create(&thread2, 128000, (void*) &switch_off, (void*) &ctime);
        /* Create thread to handle slave error handling in OP */
        osal_thread_create(&thread1, 128000, (void*) &ecatcheck, (void*) &ctime);
        /* Start cyclic part */
        osal_thread_create(&thread3, 128000, (void*) &haptic_config, (void*) argv[1]);
//        haptic_config(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }


    ros::spin();
    RDDNode::join_ros_thread(rv);
    return 0;
}
