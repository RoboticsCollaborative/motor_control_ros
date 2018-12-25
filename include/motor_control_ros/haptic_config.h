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
//* Lock for ros interface *//
boolean inOP = FALSE;
//************************//
uint8 currentgroup = 0;

uint16 motor1 = 0, motor2 = 0, psensor = 0;
double ActualPosition1 = 0, ActualVelocity1 = 0, InputTorque1 = 0;
int32 ReferencePosition1 = 0, ReferencePosition2 = 0;
double ActualPosition2 = 0, ActualVelocity2 = 0, InputTorque2 = 0;
double tau_p1 = 0, tau_p2 = 0;
double LoadPosition = 0; int32  LoadVelocity = 0;
//static double xd = 0, xd1 = 0, xd2 = 0;

/* Tempory test */
//int time_stamp[40000], cycle_stamp[40000];
//double traj[40000];

/********************************************************************************************/
/* Torque Mode */
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

/* CSP Mode */
typedef struct PACKED
{
    uint16 stat_wd;	// Status word (0x6041)
    int32 act_pos;	// Position actual value (0x6064)
    int32 pos_err;	// Position error (0x60F4)
    int32 act_vel;	// Actual velocity (0x606C)
    int16 act_tau;	// Torque actual value (0x6077)
    int32 load_pos;	// Load encoder position (0x2242)
    int32 load_vel;	// Load encoder velocity (0x2231)
} in_motor_p;

typedef struct PACKED
{
    uint16 ctrl_wd;	// Control word (0x6040)
    int32 tg_pos;	// Target position (0x607A)
    int32 vel_off;	// Velocity offset (velocity feedforward) (0x60B1)
    int32 tau_off;	// Torque offset (acceleration feedforward) (0x60B2)
} out_motor_p;
PACKED_END

/* Pressure Sensor (EL3102) */
typedef struct PACKED
{
    uint8 stat1;
    int16 val1;
    uint8 stat2;
    int16 val2;
} in_pressure_s;
PACKED_END

/* Test (EL3702) */
/*
typedef struct PACKED
{
    uint16 cyc1;
    int16 val1;
    uint16 cyc2;
    int16 val2;
    uint32 latch;
} in_test_s;
PACKED_END
*/
/********************************************************************************************/


struct sched_param schedp;

void haptic_config(void *ifnameptr);
OSAL_THREAD_FUNC ecatcheck( void *ptr );
OSAL_THREAD_FUNC switch_off( void *ptr );
OSAL_THREAD_FUNC_RT ecatthread( void *ptr );

#endif //_HAPTIC_CONFIG_H
