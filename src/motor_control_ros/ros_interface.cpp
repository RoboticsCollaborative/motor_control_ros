/**
 * \file
 * \Function interface for ros control.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  11/6/2018
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
extern "C" {
#include <soem/ethercat.h>
#include "motor_control_ros/haptic_config.h"
}


//int motor1 = 1;


/** Get actual position
 * @param[in] slave	= Slave number
 * return actual position
 */
double getActualPosition (uint slave = 1)
{
    in_motor_t *in_motor = (in_motor_t *)ec_slave[slave].inputs;
    return in_motor->ac_pos;
}


/** Get actual velocity
 * @param[in] slave	= slave number
 * return actual velocity
 */
double getActualVelocity (uint slave = 1)
{
    in_motor_t *in_motor = (in_motor_t *)ec_slave[slave].inputs;
    return in_motor->ac_vel;
}


/** Set target torque
 * @param[in] slave	= Slave number
 * @param[in] value	= Torque value
 */
int setTargetTorque (uint slave, int16 value)
{
    out_motor_t *out_motor = (out_motor_t *)ec_slave[slave].outputs;
    out_motor->tg_tau = value;
    return 0;
}
