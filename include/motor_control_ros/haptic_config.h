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
#include <pthread.h>
#include <sys/time.h>
#include <sched.h>
#include <time.h>

#include <soem/ethercat.h>
//#include "motor_control_ros/ethercat.h"

#define EC_TIMEOUTMON 500
#define NSEC_PER_SEC 1000000000

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
OSAL_THREAD_HANDLE thread1, thread2, thread3, RTthread;

char IOmap[4096];
int expectedWKC;
int64 toff;
boolean needlf;
volatile int wkc;
int dorun = 0;
//* Lock for ros interface *//
boolean inOP = FALSE;
//************************//
uint8 currentgroup = 0;

uint16 motor1, motor2;
double ActualPosition1 = 0, ActualVelocity1 = 0, InputTorque1 = 0, ReferencePosition1 = 0;
double ActualPosition2 = 0, ActualVelocity2 = 0, InputTorque2 = 0, ReferencePosition2 = 0;

/* Motors */
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

struct sched_param schedp;

void haptic_config(void *ifnameptr);
OSAL_THREAD_FUNC ecatcheck( void *ptr );
OSAL_THREAD_FUNC switch_off( void *ptr );
OSAL_THREAD_FUNC_RT ecatthread( void *ptr );

#endif //_HAPTIC_CONFIG_H
