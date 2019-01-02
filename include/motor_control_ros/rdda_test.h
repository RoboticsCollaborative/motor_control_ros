/**
 * \file
 * \Headerfile for haptic_config.c.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  9/13/2018
 */

#ifndef RDDA_TEST_H
#define RDDA_TEST_H

#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <sched.h>
#include <time.h>
#include <osal.h>

#include <soem/ethercat.h>


#define EC_TIMEOUTMON 500 /* what does this mean? */
#define NSEC_PER_SEC 1000000000


/* big mess of global variables...SAD! */

//pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
OSAL_THREAD_HANDLE thread1, thread2, thread3, RTthread;

extern char IOmap[4096];
extern int expectedWKC;
extern int64 toff;
extern boolean needlf;
extern volatile int wkc;

/* Lock for ros interface */
extern boolean inOP;

extern uint8 currentgroup;
extern uint16 motor1;
extern uint16 motor2;
extern uint16 psensor;

extern double theta1_rad;
extern double theta1_dot_radHz;
extern int32 initial_theta1_cnts; 

extern double theta2_rad;
extern double theta2_dot_radHz;
extern int32 initial_theta2_cnts; 

extern double tau_p1_Nm;
extern double tau_p2_Nm;
extern double theta1_load_rad; 
extern double theta1_dot_load_radHz;


/* BEL drive CSP Mode inputs to master */
typedef struct PACKED {
    uint16 stat_wd; // Status word (0x6041)
    int32 act_pos;  // Position actual value (0x6064)
    int32 pos_err;  // Position error (0x60F4)
    int32 act_vel;  // Actual velocity (0x606C)
    int16 act_tau;  // Torque actual value (0x6077)
    int32 load_vel; // Load encoder velocity (0x2231)
    int32 load_pos; // Load encoder position (0x2242)
} in_motor_p;


/* BEL drive CSP outputs from master */
typedef struct PACKED {
    uint16 ctrl_wd; // Control word (0x6040)
    int32 tg_pos;   // Target position (0x607A)
    int32 vel_off;  // Velocity offset (0x60B1)
    int16 tau_off;  // Torque offset (0x60B2)
} out_motor_p;


/* EL3102 pressure sensor inputs to master */
typedef struct PACKED {
    uint8 stat1;
    int16 val1;
    uint8 stat2;
    int16 val2;
} in_pressure_s;

/* there are no outputs to the EL3102 from the master */

struct sched_param schedp;

void rdda_ecat_config(void *ifnameptr);
void add_timespec(struct timespec *ts, int64 addtime);
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);

OSAL_THREAD_FUNC rdda_cyclic( void *ptr );
OSAL_THREAD_FUNC_RT ecatcheck( void *ptr );
OSAL_THREAD_FUNC switch_off( void *ptr );

#endif /* RDDA_TEST_H */



