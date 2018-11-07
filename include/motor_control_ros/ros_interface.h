/**
 * \file
 * \Headerfile for ros_interface.cpp.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  11/6/2018
 */

#ifndef _ROS_INTERFACE_H
#define _ROS_INTERFACE_H

#include <stdio.h>
#include <inttypes.h>

#include <soem/ethercat.h>

double getActualPosition (uint slave);
double getActualVelocity (uint slave);
int setTargetTorque (uint slave, int16 value);

#endif //_ROS_INTERFACE_H

