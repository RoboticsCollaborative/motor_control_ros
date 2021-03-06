/**
 * \file
 * \Headerfile for haptic_config.c.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  9/13/2018
 */

#ifndef _HAPTIC_CONFIG_H
#define _HAPTIC_CONFIG_H

#include <stdio.h>

#include <soem/ethercat.h>
//#include "motor_control_ros/ethercat.h"

#define EC_TIMEOUTMON 500

OSAL_THREAD_HANDLE thread1, thread2, thread3;

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
//* Lock for ros interface *//
boolean inOP = FALSE;
//************************//
uint8 currentgroup = 0;
double ActualPosition = 0, ActualVelocity = 0, InputTorque = 0, ReferencePosition = 0;

typedef struct PACKED
{
    int32 ac_pos; // Actual position (6064)
    int32 ac_vel; // Acutal velocity (606C)
} in_motor_t;
PACKED_END

typedef struct PACKED
{
    int32 tg_pos; // Target position (607A)
    int16 tg_tau; // Target torque (6071)
} out_motor_t;
PACKED_END

void haptic_config(void *ifnameptr);
OSAL_THREAD_FUNC ecatcheck( void *ptr );
OSAL_THREAD_FUNC switch_off( void *ptr );

#endif //_HAPTIC_CONFIG_H
