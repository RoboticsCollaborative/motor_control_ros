/**
 * \file
 * \Main function for running EtherCAT threads.
 *
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  9/13/2018
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
extern "C" {
#include "motor_control_ros/haptic_config.h"
}


int main(int argc, char *argv[])
{

    printf("SOEM (Simple Open EtherCAT Master)\nHaptic Run");

    if (argc > 1)
    {
//        printf("%d", test());
        /* Create thread to shut off motor drive */
        osal_thread_create(&thread2, 128000, (void*) &switch_off, (void*) &ctime);
        /* Create thread to handle slave error handling in OP */
        osal_thread_create(&thread1, 128000, (void*) &ecatcheck, (void*) &ctime);
        /* Start cyclic part */
        haptic_config(argv[1]);
    }
    else
    {
	    printf("Usage: haptic_run ifname1\nifname = eth1 for example\n");
    }

    printf("End program\n");
    return 0;
}


