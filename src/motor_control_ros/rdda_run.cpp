/**
 * \file
 * \Main function for running EtherCAT threads.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  9/13/2018
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
//#include <pthread.h>

#include <sched.h>

extern "C" {
#include "motor_control_ros/rdda_test.h"
}

#define STACK_SIZE 128000

int main(int argc, char *argv[]) {

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1) {
        int ctime_rt = 500; // 500us cycle time
        int ctime = 2000;   // 2000us cycle time

        /* Create RT thread (priority = 40) for PDO transfer */
        osal_thread_create_rt(&RTthread, STACK_SIZE, (void*) &rdda_cyclic, (void*) &ctime_rt);
        /* Deploy Core-Iso to ecatthread */
        cpu_set_t CPU3;
        CPU_ZERO(&CPU3);
        CPU_SET(3, &CPU3);
        pthread_setaffinity_np(RTthread, sizeof(CPU3), &CPU3);

        /* Create thread to shut off motor drive */
        osal_thread_create(&thread2, STACK_SIZE, (void*) &switch_off, (void*) &ctime);
        /* Create thread to handle slave error handling in OP */
        osal_thread_create(&thread1, STACK_SIZE, (void*) &ecatcheck, (void*) &ctime);
        
        /* Configure ethercat network and slaves */
        rdda_ecat_config(argv[1]);
    }
    else {
        printf("Usage: haptic_run ifname1\nifname = eth1 for example\n");
    }

    printf("End program\n");
    return 0;
}


