/**
 * \file
 * \Rdda reentrant functions, which may include controllers, filters and saturations.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  1/15/2019
 */

#include <stdio.h>
#include <math.h>
#include <soem/ethercat.h>

#include "motor_control_ros/rdda_type.h"
#include "motor_control_ros/rdda_funcs.h"


/** Torque saturation
 * 
 * @param[in] tau = Generated torque from controllers.
 * return saturated values.
 */
double tau_sat()

