/**
 * \file
 * \Configuration for running EtherCAT threads.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  9/10/2018
 */

#include <stdio.h>
#include <string.h>
//#include <inttypes.h>
#include <math.h>
#include <soem/ethercat.h>

#include "motor_control_ros/init_BEL.h"
#include "motor_control_ros/rdda_test.h"
#include "motor_control_ros/soem_wrappers.h"


#define COUNTS_PER_RADIAN   52151.8917
#define COUNTS_PER_REV      327680
#define LOAD_COUNTS_PER_REV 40000
#define UNITS_PER_NM        5000
#define MAX_NM              5.0
#define PASCAL_PER_COUNT    21.04178
#define NM_PER_PASCAL       2.822e-6

#define KF_GAIN             12
#define OMEGA_Z             4
#define N_SCALE_GAIN        .16

#define INERTIAL_1	(1.1051e-3 / 1.0)
#define INERTIAL_2  (INERTIAL_1 / 1.0)
#define DAMPING 	0.0
#define FINGER_DAMPING 0.01//0.014289//0.028578 // 0.02
#define FINGER_STIFFNESS 0.03//0.04494//0.08988 // 0.1
#define FINGER_STIFFNESS1 0.03//0.04494
#define FINGER_STIFFNESS2 0.03//0.04494 
#define HYDRAULIC_STIFFNESS 12.76140 // 10.0
#define HYDRAULIC_DAMPING 0.0092573 // 0.05
#define CUTOFFFREQ_A 20
#define CUTOFFFREQ_B 20
#define CUTOFFFREQ_C 20
#define CUTOFFFREQ_D 20
#define KP 0.0
#define KD 0.0
#define KA 0.0
#define MAX_INNERLOOP_NM 0.3
#define SIGMA 400.0 // 400.0
#define FC 0.016 // 0.032
#define FBIAS 0.0

#define THETA_DIFF 0.5

char IOmap[4096];
int expectedWKC;
int loop_num=0;
int64 toff;
boolean needlf;
volatile int wkc;

//* Lock for ros interface *//
boolean inOP = FALSE;

uint8 currentgroup = 0;
uint16 motor1 = 0;
uint16 motor2 = 0;
uint16 psensor = 0;

double theta1_rad = 0.0;
double theta1_dot_radHz = 0.0;
int32 initial_theta1_cnts = 0; 

double theta2_rad = 0.0;
double theta2_dot_radHz = 0.0;
int32 initial_theta2_cnts = 0; 

double tau_p1_Nm = 0;
double tau_p2_Nm = 0;
//double theta1_load_rad = 0.0; 
//double theta1_dot_load_radHz = 0.0;
double theta2_load_rad = 0.0; 
double theta2_dot_load_radHz = 0.0;
int16 act_tau2 = 0.0;


int16 tau_sat_units_1 = 0;
int16 tau_sat_units_2 = 0;

int64 time_usec = 0;

/* filter states */
double xx1 = 0.0;
double yy1 = 0.0;
/* DOB parameters */
double comp_p1_Nm = 0.0, comp_p2_Nm = 0.0;
double vel_back_1 = 0.0, vel_back_2 = 0.0;
double lambda_A = 2.0 * M_PI * CUTOFFFREQ_A;
double lambda_B = 2.0 * M_PI * CUTOFFFREQ_B;
double lambda_C = 2.0 * M_PI * CUTOFFFREQ_C;
double lambda_D = 2.0 * M_PI * CUTOFFFREQ_D;
double acc_1 = 0.0, acc_2 = 0.0;
double f_n1 = 0.0, f_n2 = 0.0;
double f_n1_back = 0.0, f_n2_back = 0.0;
double f_fn1 = 0.0, f_fn2 = 0.0;
double f_fn1_back = 0.0, f_fn2_back = 0.0;
double f_ref1 = 0.0, f_ref2 = 0.0;
double f_z1 = 0.0, f_z2 = 0.0;
double f_z1_back = 0.0, f_z2_back = 0.0;
double f_in1 = 0.0, f_in2 = 0.0;
double f_in1_back = 0.0, f_in2_back = 0.0;
double f_int1 = 0.0, f_int2 = 0.0;
double p1 = 0.0, p2 = 0.0;
double p1_back = 0.0, p2_back = 0.0;
double tau_p1_back = 0.0, tau_p2_back = 0.0;
double tau_p1_bback = 0.0, tau_p2_bback = 0.0;
double p1c = 0.0, p2c = 0.0;
double p1c_back = 0.0, p2c_back = 0.0;
double p1c_bback = 0.0, p2c_bback = 0.0;
double f_a1 = 0.0, f_a2 = 0.0;
double f_a1_back = 0.0, f_a2_back = 0.0;
double f_fa1 = 0.0, f_fa2 = 0.0;
double f_fa1_back = 0.0, f_fa2_back = 0.0;
double vel_finger_1 = 0.0, vel_finger_2 = 0.0;
double vel_finger_11 = 0.0, vel_finger_21 = 0.0;
double vel_finger_11_back = 0.0, vel_finger_21_back = 0.0;
double f_hy_1 = 0.0, f_hy_2 = 0.0;
double f_hy_1_back = 0.0, f_hy_2_back = 0.0;
double f_fhy_1 = 0.0, f_fhy_2 = 0.0;
double f_fhy_1_back = 0.0, f_fhy_2_back = 0.0;

void rdda_ecat_config(void *ifnameptr)
{
    char *ifname = (char *) ifnameptr;
    needlf = FALSE;
    inOP = FALSE;
    struct timespec ts, tleft;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);  
  
    /* Conver from timeval to timespec */
    //ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    //ts.tv_nsec = ht * 1000000;
    //cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    //toff = 0;
        
    printf("Begin network configuration\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n",ifname);
    } 
    else { 
        printf("No socket connection on %s\nExcecute as root\n",ifname);
        return;
    }

    /* find and configure slaves */
    if ( ec_config_init(FALSE) > 0 ) {
        printf("%d slaves found and configured.\n",ec_slavecount);
    } 
    else {
        printf("No slaves found!\n");
        printf("Close socket\n");
        ec_close(); /* stop SOEM, close socket */
        return;
    }

    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    int slave; /* slave = 0 is master, slave = 1 is first slave, slave =2 is second slave, etc. */

    /* locate slaves */
    for (slave = 1; slave <= ec_slavecount; slave ++){
     
        /* BEL drives */
        if ((ec_slave[slave].eep_man == 0x000000ab) && (ec_slave[slave].eep_id == 0x00001110)){   
            
            uint32 serial_num;
            READ_SDO(slave, 0x1018, 4, serial_num, "Amplifier's Serial Number");
            
            
            /* motor 1 */
            if (0x2098302 == serial_num){
                motor1 = slave;
                /* CompleteAccess disabled for Bel driver */
                ec_slave[motor1].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[motor1].name, motor1);
                map_BEL_CSP_callback(motor1);
            }
            /* motor 2 */
            if (0x2098303 == serial_num) {
                motor2 = slave;
                /* CompleteAccess disabled for Bel driver */
                ec_slave[motor2].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[motor2].name, motor2);
                map_BEL_CSP_callback(motor2);
            }
        }
        /* EL3102 */
        if ((ec_slave[slave].eep_man == 0x00000002) && (ec_slave[slave].eep_id == 0x0c1e3052) && (slave == 4)) { 
            psensor = slave;
        }
    } /* end for loop over slaves */

    /* If Complete Access (CA) disabled => auto-mapping work */
    ec_config_map(&IOmap);

    /* Let DC off for the time being */
    //ec_configdc();

    printf("Slaves mapped, state to SAFE_OP.\n");
    /* wait for all slaves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

    printf("Request operational state for all slaves\n");
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /**
     * Initialize motor parameters
     */
    init_BEL1_CSP(motor1);
    init_BEL2_CSP(motor2);

    /* Initialize reference position */
    READ_SDO(motor1, 0x6064, 0, initial_theta1_cnts, "Initial angle, motor 1");      
    READ_SDO(motor2, 0x6064, 0, initial_theta2_cnts, "Initial angle, motor 2");      
    printf("ref_pos1: %lf, ref_pos2: %lf\n", (double)initial_theta1_cnts/COUNTS_PER_RADIAN, (double)initial_theta2_cnts/COUNTS_PER_RADIAN);


        /* Activate motor drive */
//      WRITE_SDO(motor1, 0x6040, 0, BUF16, 15, "*Control word: motor1*");

   /* Going operational */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* send one valid process data to make outputs in slaves happy*/
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    /* request OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
        printf("Operational state reached for all slaves.\n");
        inOP = TRUE;
    }


    /* cyclic loop */
    /* Note that we are not in the real-time thread, but the "config"/"monitor" thread
     * so we set the inOP flag, and then loop at a slower rate and log some data to
     * stdout.  Note that if all slaves don't reach OP, we are in the same condition as
     * when we fall out of OP mode (inOP goes false from the thread monitoring if we
     * hit "q" to quit --- in both cases we want to go back to INIT and shut down.
     */ 
    if(wkc >= expectedWKC){
 
    while(inOP && (loop_num <= 121999)) {
        printf("enc2_pos: %+2.4lf, enc2_vel: %+2.4lf, th1: %+2.4lf, th2: %+2.4lf, p1: %+2.4lf, p2: %+2.4lf, n: %ld, tau: %+2.4lf\r", theta2_load_rad, theta2_dot_load_radHz, theta1_rad, theta2_rad, tau_p1_Nm, tau_p2_Nm, time_usec, (double)(tau_sat_units_1)/UNITS_PER_NM);

        /* calculate next cycle start */
//        add_timespec(&ts, 2000000);
        /* wait to cycle start */
//        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
	
	fflush(stdout);	
        }
    } else {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        /* give some status/state information */
        for(slave = 1; slave<=ec_slavecount ; slave++){
            if(ec_slave[slave].state != EC_STATE_OPERATIONAL) {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
            }
        }
    }
	inOP = FALSE;
    usleep(10000);
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);
        
}


/* Add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
    nsec = ts->tv_nsec % NSEC_PER_SEC;
    ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
    ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* Set linux sync point 50us later than DC sync */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime/2)) { delta=delta-cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsettime = -(delta/100)-(integral/20);
}

/* step function */
double fstep(double dmax, double dmin, double current_time)
{
    /* function parameters */
    double t0 = 2.0; // time to begin cycles
    double dtw = 6.0; // wait time
    double dto = 0.3; // open time
    double dth = 2.0; // hold time
    double dtc = 1.0; // close time
    
    double T = 0.0;
    T = dtw + dto + dth + dtc;
    
    double local_time = 0.0;
    
    if (current_time < t0) {
        return dmax;
    }
    else {
        local_time = fmod(current_time - t0, T);
        if(local_time < dtw) {
            return dmax;
        }
        else if(local_time < (dtw + dto)) {
            return (dmax - dmin) / 2.0 * cos((local_time - dtw) * M_PI / dto) + (dmax + dmin) / 2.0;
        }
        else if(local_time < (dtw + dto + dth)) {
            return dmin;
        }
        else {
            return -1.0 * (dmax - dmin) / 2.0 * cos((local_time - (dtw + dto + dth)) * M_PI / dtc) + (dmax + dmin) / 2.0;
        }
    }
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT rdda_cyclic( void *ptr )
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    double tau_1, tau_2, chirp;
    
    /* filter coefficients */
    double a1 = 0.0;
    double b0 = 0.0;
    double b1 = 0.0;
    double ab1 = 0.0, bb0 = 0.0, bb1 = 0.0;
    double aa1 = 0.0, ba0 = 0.0, ba1 = 0.0;
    double ac1 = 0.0, bc0 = 0.0, bc1 = 0.0;
    double ad1 = 0.0, bd0 = 0.0, bd1 = 0.0;
    double acmp1 = 0.0, acmp2 = 0.0;
    double bcmp0 = 0.0, bcmp1 = 0.0, bcmp2 = 0.0;
    double ahe1 = 0.0, bhe0 = 0.0, bhe1 = 0.0;


    /* create a data file */
    FILE *fptr;
    char filename[] = "/home/ethercat-master/rdda.dat";
    remove(filename);
    fptr = fopen(filename, "w");


    /* Time stamp */
/*
//    static uint32 Time0 = 0;
//    static uint32 Time1 = 0;
//    static uint32 Time_cycle = 0;

//    pthread_mutex_lock(&mutex);
//    gettimeofday(&ts, NULL);
*/

    clock_gettime(CLOCK_MONOTONIC, &ts);  
  
    /* Conver from timeval to timespec */
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    toff = 0;

    float Ts = *(int*)ptr*1e6; /* convert cycle interval to units of seconds */
    float a = OMEGA_Z/N_SCALE_GAIN;
    float b = OMEGA_Z;
    float K = KF_GAIN;
    
    /* chirp params */
    double f1 = 0.01; /* initial frequency 0.1Hz */
    double f2 = 1000; /* final frequency 200Hz */
    double T = 60; /* period 10s */
    double t = 0.0; /* time for chirp wave */
    double dt = 0.5e-3; /* sample time 500us */
    
    /* calculate phase-lag filter coefficients before spawning real time thread */
    //a1 = -1.0*(a*Ts-2.0)/(a*Ts+2.0);
    //b0 = K*(b*Ts+2.0)/(a*Ts+2.0);
    //b1 = K*(b*Ts-2.0)/(a*Ts+2.0);
    
    /* QB/QA */
    a1 = -1.0 * (lambda_B * dt - 2.0) / (2.0 + lambda_B * dt);
    b0 = (2.0 + lambda_A * dt) * lambda_B / (2.0 + lambda_B * dt) / lambda_A;
    b1 = (lambda_A * dt - 2.0) * lambda_B / (2.0 + lambda_B * dt) / lambda_A;
    
    /* QA */
    aa1 = -1.0 * (lambda_A * dt - 2.0) / (2.0 + lambda_A * dt);
    ba0 = lambda_A * dt / (2.0 + lambda_A * dt);
    ba1 = lambda_A * dt / (2.0 + lambda_A * dt);
    
    /* QB */
    ab1 = -1.0 * (lambda_B * dt - 2.0) / (2.0 + lambda_B * dt);
    bb0 = lambda_B * dt / (2.0 + lambda_B * dt);
    bb1 = lambda_B * dt / (2.0 + lambda_B * dt);  
    
    /* QC */
    ac1 = -1.0 * (lambda_C * dt - 2.0) / (2.0 + lambda_C * dt);
    bc0 = lambda_C * dt / (2.0 + lambda_C * dt);
    bc1 = lambda_C * dt / (2.0 + lambda_C * dt);
    
    /* QD */
    ad1 = -1.0 * (lambda_D * dt - 2.0) / (2.0 + lambda_D * dt);
    bd0 = lambda_D * dt / (2.0 + lambda_D * dt);
    bd1 = lambda_D * dt / (2.0 + lambda_D * dt);
    
    /* pressure part of finger damping compensation filter coefficient */
    acmp1 = -1.0 * ((lambda_A * dt - 2.0) * (2.0 * HYDRAULIC_DAMPING + HYDRAULIC_STIFFNESS * dt) + (2.0 + lambda_A * dt) * (HYDRAULIC_STIFFNESS * dt - 2.0 * HYDRAULIC_DAMPING)) / ((lambda_A * dt + 2.0) * (2.0 * HYDRAULIC_DAMPING + HYDRAULIC_STIFFNESS * dt));     
    acmp2 = -1.0 * (lambda_A * dt - 2.0) * (HYDRAULIC_STIFFNESS * dt - 2.0 * HYDRAULIC_DAMPING) / ((lambda_A * dt + 2.0) * (2.0 * HYDRAULIC_DAMPING + HYDRAULIC_STIFFNESS * dt));
    bcmp0 = lambda_A * dt * (2.0 * FINGER_DAMPING + FINGER_STIFFNESS * dt) / ((lambda_A * dt + 2.0) * (2.0 * HYDRAULIC_DAMPING + HYDRAULIC_STIFFNESS * dt));
    bcmp1 = lambda_A * dt * 2.0 * FINGER_STIFFNESS * dt / ((lambda_A * dt + 2.0) * (2.0 * HYDRAULIC_DAMPING + HYDRAULIC_STIFFNESS * dt));
    bcmp2 = lambda_A * dt * ( -2.0 * FINGER_DAMPING + FINGER_STIFFNESS * dt) / ((lambda_A * dt + 2.0) * (2.0 * HYDRAULIC_DAMPING + HYDRAULIC_STIFFNESS * dt));

    
    /* filter coefficient for equivalent finger velocity */
    ahe1 = -1.0 * (HYDRAULIC_STIFFNESS * dt - 2.0 * HYDRAULIC_DAMPING) / (HYDRAULIC_STIFFNESS * dt + 2.0 * HYDRAULIC_DAMPING);
    bhe0 = 2.0 / (HYDRAULIC_STIFFNESS * dt + 2.0 * HYDRAULIC_DAMPING);
    bhe1 = -2.0 / (HYDRAULIC_STIFFNESS * dt + 2.0 * HYDRAULIC_DAMPING);

    /* reference parameters */
    double max_theta_1 = 0.0;
    double max_theta_2 = 0.0;
    double min_theta_1 = 0.0;
    double min_theta_2 = 0.0;
    double ref_theta_1 = 0.0;
    double ref_theta_2 = 0.0;
    double ref_theta_1_back = 0.0;
    double ref_theta_2_back = 0.0;
    double ref_vel_theta_1 = 0.0;
    double ref_vel_theta_2 = 0.0;
 
    /* Busy waiting for OP mode */
    while(!inOP){}

    /* motor1 */
    out_motor_p *out_motor1 = (out_motor_p *)ec_slave[motor1].outputs;
    in_motor_p  *in_motor1  = (in_motor_p *) ec_slave[motor1].inputs;
    
    /* motor2 */
    out_motor_p *out_motor2 = (out_motor_p *)ec_slave[motor2].outputs;
    in_motor_p  *in_motor2  = (in_motor_p *) ec_slave[motor2].inputs;

    /* pressure sensor */
    in_pressure_s *in_pressure = (in_pressure_s *)ec_slave[psensor].inputs; 

    /* pressure sensor */
    //comp_p1_Nm = (double)(in_pressure->val1) * PASCAL_PER_COUNT * NM_PER_PASCAL;
    //comp_p2_Nm = (double)(in_pressure->val2) * PASCAL_PER_COUNT * NM_PER_PASCAL;
    comp_p1_Nm = 0.04;
    comp_p2_Nm = 0.04;

    // motor1
    out_motor1->ctrl_wd = (uint16)0;
    out_motor1->tg_pos = initial_theta1_cnts;
    out_motor1->vel_off = (int32)0;
    out_motor1->tau_off = (int16)0;
    
    theta1_rad = (double)(in_motor1->act_pos)/COUNTS_PER_RADIAN;
    theta1_dot_radHz = (double)(in_motor1->act_vel)/COUNTS_PER_RADIAN/10.0;
 
    // motor2
    out_motor2->ctrl_wd = (uint16)0;
    out_motor2->tg_pos = initial_theta2_cnts;
    out_motor2->vel_off = (int32)0;
    out_motor2->tau_off = (int16)0;

    theta2_rad = (double)(in_motor2->act_pos)/COUNTS_PER_RADIAN;
    theta2_dot_radHz = (double)(in_motor2->act_vel)/COUNTS_PER_RADIAN/10.0;

    act_tau2 = (int16)in_motor2->act_tau;
    
    /* assume that initial force is zero. */
    xx1 = 0.0;
    yy1 = 0.0;
    tau_1 = 0.0;
    tau_2 = 0.0;
    
    /* fire the PDOs! */
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    time_usec = 0;
    
    /* initialization for reference */
    max_theta_1 = theta1_rad;
    min_theta_1 = theta1_rad - THETA_DIFF;
    ref_theta_1 = theta1_rad;
    ref_theta_1_back = theta1_rad;
    max_theta_2 = theta2_rad;
    min_theta_2 = theta2_rad - THETA_DIFF;
    ref_theta_2 = theta2_rad;
    ref_theta_2_back = theta2_rad;
    
    ref_vel_theta_1 = theta1_dot_radHz;
    ref_vel_theta_2 = theta2_dot_radHz;
    vel_back_1 = theta1_dot_radHz;
    vel_back_2 = theta2_dot_radHz;
    
    tau_p1_bback = (double)(in_pressure->val1) * PASCAL_PER_COUNT * NM_PER_PASCAL - comp_p1_Nm;
    tau_p1_back = tau_p1_bback;
    p1 = tau_p1_back;
    p1_back = tau_p1_back;
    p1c = p1;
    p1c_back = p1;
    p1c_bback = p1;
    tau_p2_bback = (double)(in_pressure->val2) * PASCAL_PER_COUNT * NM_PER_PASCAL - comp_p2_Nm;
    tau_p2_back = tau_p2_bback;
    p2 = tau_p2_back;
    p2_back = tau_p2_back;
    p2c = p2;
    p2c_back = p2;
    p2c_bback = p2;
    
    
    while(inOP) {
        
        loop_num++;
        
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
    
        /* Time stamp */               
        time_usec = osal_current_time().sec*1000000 + osal_current_time().usec;

/*
        //Time_cycle = Time0 - Time1;
        //Time1 = Time0;
        //time_stamp[i] = Time0;
        //cycle_stamp[i] = Time_cycle;
*/
    
        /* Cyclic data */
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
            
        /* pressure sensor */
        tau_p1_Nm = (double)(in_pressure->val1) * PASCAL_PER_COUNT * NM_PER_PASCAL - comp_p1_Nm;
        tau_p2_Nm = (double)(in_pressure->val2) * PASCAL_PER_COUNT * NM_PER_PASCAL - comp_p2_Nm;
    
   
 /*========================================================================================================*/     
        /* calculate force feedback */
 
	/* For tests */ 
	/* Note that we generate a chirp sine wave on torque offset on one finger with frequency
	   range from 0.1Hz to 200Hz, the output data plot in matlab shows the response for a 
	   passive system. 
	   There're 4 cases covered here:
	   a. Force feedforward (passive);
	   b. Phase lead;
	   c. Disturbance observer;
	   d. Natural admittance compensation.
	*/

	/* chirp wave */
    if (loop_num < 2000) chirp = 0.0;
    else {
        t = (double)((loop_num - 2000)*dt);
        chirp = 0.1 * sin( 2*M_PI*f1*T/log(f2/f1) * (exp(t/T*log(f2/f1))-1) );  //0.3 0.2 0.12 0.1
    }

      
	/* force feedforward */
//        tau = tau_p1_Nm * 1.5;

//	tau_2 = chirp;

        
        /* phase lag/lead force feedback */
/*
        tau = a1*yy1 + b0*tau_p2_Nm + b1*xx1;
        
        xx1 = tau_p2_Nm;
        yy1 = tau;
*/

	/* Step reference calculator */
/*	ref_theta_1 = fstep(max_theta_1, min_theta_1, t);
	ref_vel_theta_1 = (ref_theta_1 - ref_theta_1_back) / dt;
	ref_theta_1_back = ref_theta_1;
	ref_theta_2 = fstep(max_theta_2, min_theta_2, t);
	ref_vel_theta_2 = (ref_theta_2 - ref_theta_2_back) / dt;
	ref_theta_2_back = ref_theta_2;
*/

    /* Sinusoid reference*/
//    ref_theta_1 = THETA_DIFF * sin(t);
//    ref_theta_1 = chirp;
        
	/* Disturbance observer */
    /* motor 1 */
    f_ref1 = 0.0;
	acc_1 = (theta1_dot_radHz - vel_back_1) / dt;
	vel_back_1 = theta1_dot_radHz;

    f_z1 = KP *(ref_theta_1 - theta1_rad) + KD * (ref_vel_theta_1 - theta1_dot_radHz) + KA * (0.0 - acc_1);
    f_in1 = bc0 * f_z1 + bc1 * f_z1_back + ac1 * f_in1_back;
    f_z1_back = f_z1;
    f_in1_back = f_in1;
    
	f_n1 = INERTIAL_1 * acc_1 + (DAMPING - FINGER_DAMPING) * theta1_dot_radHz - FINGER_STIFFNESS1 * (theta1_rad - max_theta_1);
    f_fn1 = bb0 * f_n1 + bb1 * f_n1_back + ab1 * f_fn1_back;
    f_n1_back = f_n1;
    f_fn1_back = f_fn1;
    
    p1 = bd0 * tau_p1_Nm + bd1 * tau_p1_back + ad1 * p1_back;
    p1c = bcmp0 * tau_p1_Nm + bcmp1 * tau_p1_back + bcmp2 * tau_p1_bback + acmp1 * p1c_back + acmp2 * p1c_bback;
    p1_back = p1;
    p1c_bback = p1c_back;
    p1c_back = p1c;
    
    vel_finger_11 = bhe0 * tau_p1_Nm + bhe1 * tau_p1_back + ahe1 * vel_finger_11_back;
    vel_finger_11_back = vel_finger_11;
    vel_finger_1 = vel_finger_11 + theta1_dot_radHz;
    tau_p1_bback = tau_p1_back;
    tau_p1_back = tau_p1_Nm;
//    p1c = 0.0;

    f_hy_1 = (f_hy_1_back + dt * SIGMA * (vel_finger_1 * FC - fabs(vel_finger_1) * FBIAS)) / (1.0 + dt * SIGMA * fabs(vel_finger_1));
//    f_hy_1 = (f_hy_1_back + SIGMA * FC * dt * vel_finger_1 * (1.0 - 2.0 * fabs(vel_finger_1) / vel_finger_1 * FBIAS / FC + (FBIAS / FC) * (FBIAS / FC))) / (1.0 + SIGMA * FC * dt * vel_finger_1 * (2.0 * fabs(vel_finger_1) / vel_finger_1 / FC - (f_hy_1_back + 2.0 * FBIAS) / (FC * FC)));
//    f_hy_1 = f_hy_1_back + dt * SIGMA * FC * vel_finger_1 * pow((1.0 - fabs(vel_finger_1) / vel_finger_1 * (f_hy_1_back + FBIAS) / FC), 3.0);
    f_fhy_1 = bd0 * f_hy_1 + bd1 * f_hy_1_back + ad1 * f_fhy_1_back;
    f_fhy_1_back = f_fhy_1;
    f_hy_1_back = f_hy_1;
//    f_fhy_1 = 0.0;

	f_int1 = f_int1 + lambda_A * dt * (f_ref1 + f_in1 + p1 + p1c + f_fhy_1 - f_fn1);
    tau_1 = (f_ref1 + f_in1 + p1 + p1c + f_fhy_1 - f_fn1) + f_int1;
   
    f_a1 = tau_1;
    f_fa1 = ba0 * f_a1 + ba1 * f_a1_back + aa1 * f_fa1_back;
    f_a1_back = f_a1;
    f_fa1_back = f_fa1;
    
    if ((f_fa1 - f_fn1)  > MAX_INNERLOOP_NM){
        tau_1 = MAX_INNERLOOP_NM + p1 + p1c + f_fhy_1 + f_ref1 + f_in1;
        f_a1 = tau_1;
        f_fa1 = ba0 * f_a1 + ba1 * f_a1_back + aa1 * f_fa1_back;
        f_a1_back = f_a1;
        f_fa1_back = f_fa1;
    }
    else if ((f_fa1 - f_fn1) < -1.0 * MAX_INNERLOOP_NM){
        tau_1 = -1.0 * MAX_INNERLOOP_NM + p1 + p1c + f_fhy_1 + f_ref1 + f_in1;
        f_a1 = tau_1;
        f_fa1 = ba0 * f_a1 + ba1 * f_a1_back + aa1 * f_fa1_back;
        f_a1_back = f_a1;
        f_fa1_back = f_fa1;
    }
    
//    tau_1 = chirp;
    
    /* motor 2 */
    f_ref2 = 0.0;
	acc_2 = (theta2_dot_radHz - vel_back_2) / dt;
	vel_back_2 = theta2_dot_radHz;
    
    f_z2 = KP *(ref_theta_2 - theta2_rad) + KD * (ref_vel_theta_2 - theta2_dot_radHz) + KA * (0.0 - acc_2);
    f_in2 = bc0 * f_z2 + bc1 * f_z2_back + ac1 * f_in2_back;
    f_z2_back = f_z2;
    f_in2_back = f_in2;
    
	f_n2 = INERTIAL_2 * acc_2 + (DAMPING - FINGER_DAMPING) * theta2_dot_radHz - FINGER_STIFFNESS2 * (theta2_rad - max_theta_2);
    f_fn2 = bb0 * f_n2 + bb1 * f_n2_back + ab1 * f_fn2_back;
    f_n2_back = f_n2;
    f_fn2_back = f_fn2;
    
    p2 = bd0 * tau_p2_Nm + bd1 * tau_p2_back + ad1 * p2_back;
    p2c = bcmp0 * tau_p2_Nm + bcmp1 * tau_p2_back + bcmp2 * tau_p2_bback + acmp1 * p2c_back + acmp2 * p2c_bback;
    p2_back = p2;
    p2c_bback = p2c_back;
    p2c_back = p2c;
    
    vel_finger_21 = bhe0 * tau_p2_Nm + bhe1 * tau_p2_back + ahe1 * vel_finger_21_back;
    vel_finger_21_back = vel_finger_21;
    vel_finger_2 = vel_finger_21 + theta2_dot_radHz;
    tau_p2_bback = tau_p2_back;
    tau_p2_back = tau_p2_Nm;
//    p2c = 0.0;

    f_hy_2 = (f_hy_2_back + dt * SIGMA * (vel_finger_2 * FC - fabs(vel_finger_2) * FBIAS)) / (1.0 + dt * SIGMA * fabs(vel_finger_2));
//    f_hy_2 = (f_hy_2_back + SIGMA * FC * dt * vel_finger_2 * (1.0 - 2.0 * fabs(vel_finger_2) / vel_finger_2 * FBIAS / FC + (FBIAS / FC) * (FBIAS / FC))) / (1.0 + SIGMA * FC * dt * vel_finger_2 * (2.0 * fabs(vel_finger_2) / vel_finger_2 / FC - (f_hy_2_back + 2.0 * FBIAS) / (FC * FC)));
//    f_hy_2 = f_hy_2_back + dt * SIGMA * FC * vel_finger_2 * pow ((1.0 - fabs(vel_finger_2) / vel_finger_2 * (f_hy_2_back + FBIAS) / FC), 3.0);
    f_fhy_2 = bd0 * f_hy_2 + bd1 * f_hy_2_back + ad1 * f_fhy_2_back;
    f_fhy_2_back = f_fhy_2;
    f_hy_2_back = f_hy_2;
//    f_fhy_2 = 0.0;
    
	f_int2 = f_int2 + lambda_A * dt * (f_ref2 + f_in2 + p2 + p2c + f_fhy_2 - f_fn2);
    tau_2 = (f_ref2 + f_in2 + p2 + p2c + f_fhy_2 - f_fn2) + f_int2;
   
    f_a2 = tau_2;
    f_fa2 = ba0 * f_a2 + ba1 * f_a2_back + aa1 * f_fa2_back;
    f_a2_back = f_a2;
    f_fa2_back = f_fa2;
    
    if ((f_fa2 - f_fn2)  > MAX_INNERLOOP_NM){
        tau_2 = MAX_INNERLOOP_NM + p2 + p2c + f_fhy_2 + f_ref2 + f_in2;
        f_a2 = tau_2;
        f_fa2 = ba0 * f_a2 + ba1 * f_a2_back + aa1 * f_fa2_back;
        f_a2_back = f_a2;
        f_fa2_back = f_fa2;
    }
    else if ((f_fa2 - f_fn2) < -1.0 * MAX_INNERLOOP_NM){
        tau_2 = -1.0 * MAX_INNERLOOP_NM + p2 + p2c + f_fhy_2 + f_ref2 + f_in2;
        f_a2 = tau_2;
        f_fa2 = ba0 * f_a2 + ba1 * f_a2_back + aa1 * f_fa2_back;
        f_a2_back = f_a2;
        f_fa2_back = f_fa2;
    }
 
/*========================================================================================================*/     

        if (tau_1 > MAX_NM ){
            tau_sat_units_1 = (int16)(MAX_NM * (double)UNITS_PER_NM);
        }
        else if (tau_1 < (-1.0*MAX_NM) ){
            tau_sat_units_1 = (int16)(-1.0 * MAX_NM * (double)UNITS_PER_NM);
        }
        else {
            tau_sat_units_1 = (int16)(tau_1 * (double)UNITS_PER_NM);
        }
        
        if (tau_2 > MAX_NM ){
            tau_sat_units_2 = (int16)(MAX_NM * (double)UNITS_PER_NM);
        }
        else if (tau_2 < (-1.0*MAX_NM) ){
            tau_sat_units_2 = (int16)(-1.0 * MAX_NM * (double)UNITS_PER_NM);
        }
        else {
            tau_sat_units_2 = (int16)(tau_2 * (double)UNITS_PER_NM);
        }
/*
	if (loop_num > 20) {
            out_motor1->ctrl_wd = (uint16)15;
            out_motor2->ctrl_wd = (uint16)15;
	}
*/


        // motor1
        out_motor1->ctrl_wd = (uint16)15;
        out_motor1->tg_pos = initial_theta1_cnts;
//        out_motor1->tg_pos = initial_theta1_cnts + (int32)(ref_theta_1*COUNTS_PER_RADIAN);
        out_motor1->vel_off = (int32)0;
        out_motor1->tau_off = tau_sat_units_1;
//        out_motor1->tau_off = (int16)0;
        
        theta1_rad = (double)(in_motor1->act_pos)/COUNTS_PER_RADIAN;
        theta1_dot_radHz = (double)(in_motor1->act_vel)/COUNTS_PER_RADIAN/10.0;

       
        // motor2
        out_motor2->ctrl_wd = (uint16)15;
        out_motor2->tg_pos = initial_theta2_cnts;
//        out_motor2->tg_pos = initial_theta2_cnts + (int32)(chirp*COUNTS_PER_RADIAN);
        out_motor2->vel_off = (int32)0;
        out_motor2->tau_off = tau_sat_units_2;
//        out_motor2->tau_off = (int16)0;
    
        theta2_rad = (double)(in_motor2->act_pos)/COUNTS_PER_RADIAN;
        theta2_dot_radHz = (double)(in_motor2->act_vel)/COUNTS_PER_RADIAN/10.0;
    
	act_tau2 = (double)(in_motor2->act_tau)/UNITS_PER_NM;

        /* load encoder */
        theta2_load_rad = (double)(in_motor2->load_pos)*COUNTS_PER_REV/LOAD_COUNTS_PER_REV/COUNTS_PER_RADIAN;
        theta2_dot_load_radHz = (double)(in_motor2->load_vel)*COUNTS_PER_REV/LOAD_COUNTS_PER_REV/COUNTS_PER_RADIAN/10.0;

  	/* save data to file */
	fprintf(fptr, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", chirp, theta1_rad, theta2_rad, theta2_load_rad, theta1_dot_radHz, theta2_dot_radHz, theta2_dot_load_radHz, tau_p1_Nm, tau_p2_Nm);
  
        needlf = TRUE;
    }


    /* close file */
    fclose(fptr);
 
}


OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

OSAL_THREAD_FUNC switch_off( void *ptr )
{
    int c; 
    uint16 BUF16;
 
    do {
        c = getchar();
        putchar(c);
    } while (c != 'q');

    /* Disable motor drive */
    WRITE_SDO(1, 0x6040, 0, BUF16, 0, "*Control word: motor1*");
    inOP = FALSE;
    
}
