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

void start_driver(char* ifname)
{
    
    /* Create thread to handle slave error handling in OP */
    osal_thread_create(&thread1, 128000, (void*) &ecatcheck, (void*) &ctime);
    /* Create thread to shut off motor drive */
    osal_thread_create(&thread2, 128000, (void*) &switch_off, (void*) &ctime);
    /* Start cyclic part */
    osal_thread_create(&thread3, 128000, (void*) &rdda_ecat_config, (void*) ifname);

    /* Deploy iso-core to ethercat thread */
    cpu_set_t CPU3;
    CPU_ZERO(&CPU3);
    CPU_SET(3, &CPU3);
    pthread_setaffinity_np(thread3, sizeof(CPU3), &CPU3);
}

void test_ros()
{
    ros::NodeHandle node("~");
    RDDNode rdd(node);
    rdd.run();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rdd");
//    test_ros();

    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        start_driver(argv[1]);

        ros::NodeHandle node("~");
        RDDNode rdd(node);
        rdd.run();

    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    return 0;
}
