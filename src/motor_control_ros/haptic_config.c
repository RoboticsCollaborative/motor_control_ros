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
#include <inttypes.h>
#include <soem/ethercat.h>
//#include "motor_control_ros/ethercat.h"

#include "motor_control_ros/config_map.h"
#include "motor_control_ros/ecattype.h"
#include "motor_control_ros/haptic_config.h"
#include "motor_control_ros/controller.h"
#include "motor_control_ros/slave_init.h"

void haptic_config(void *ifnameptr)
{
    char *ifname = (char *) ifnameptr;
    needlf = FALSE;
    inOP = FALSE;
    
    uint16 BUF16; 
    // uint8 BUF8;  //uint32 BUF32;
	    
    printf("Starting to configure robot\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
      	printf("ec_init on %s succeeded.\n",ifname);
      	/* find and auto-config slaves */


       	if ( ec_config_init(FALSE) > 0 )
      	{
            printf("%d slaves found and configured.\n",ec_slavecount);

	    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

	    int slave;

	    /* user Def */
	    /* locate slave (BEL motor drive) */
	    if (ec_slavecount > 0)
	    {
	    	for (slave = 1; slave <= ec_slavecount; slave ++)
	    	{
		    if ((ec_slave[slave].eep_man == 0x000000ab) && (ec_slave[slave].eep_id == 0x00001110))
		    {
			motor1 = slave;	
		        /* CompleteAccess disabled for Bel driver */
	    		ec_slave[motor1].CoEdetails ^= ECT_COEDET_SDOCA;
			/* Set PDO mapping */
		    	printf("Found %s at position %d\n", ec_slave[motor1].name, motor1);
			motor_setup(motor1);
		    }
	    	}
	    }

	    /* If CA disabled => auto-mapping work */
            ec_config_map(&IOmap);

	    /* Let DC off for the time being */
//            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

	    /* Going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
            	printf("Operational state reached for all slaves.\n");
            	inOP = TRUE;

		/* Output torque and velocity values to a file */
/*
		FILE *fp = NULL;
		char *filename = "/home/protectli/velocity.dat";
		remove(filename);
		fp = fopen(filename, "w");
*/
	  	/**
	  	* Map PDO address with struct pointer
	 	*/
	     	motor_init(motor1);

		/* Initialize origin (By SDO) */
/*
		ec_send_processdata();
               	wkc = ec_receive_processdata(EC_TIMEOUTRET);
		ActualPosition = (in_motor->ac_pos)/Counts_per_radian;
		ActualVelocity = (in_motor->ac_vel)/Counts_per_radian/10;
		out_motor->tg_tau = (int16)0;
		ec_send_processdata();
             	wkc = ec_receive_processdata(EC_TIMEOUTRET);
		ReferencePosition = ActualPosition;
*/
		/* Activate motor drive */
		WRITE_SDO(motor1, 0x6040, 0, BUF16, 15, "*Control word: motor1*");
	
	        /* activate cyclic process data */
		dorun = 1;

                /* cyclic loop */
	        if(wkc >= expectedWKC)
		{
		    while(inOP && dorun)
            	    {
		        printf("ac_pos: %lf, ac_vel: %lf, cmd_tau: %lf, WKC %d , O:\r", ActualPosition, ActualVelocity, InputTorque, wkc);
                    }
		}
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(slave = 1; slave<=ec_slavecount ; slave++)
                {
                    if(ec_slave[slave].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End haptic_run, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
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

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread( void *ptr )
{
    struct timespec	ts, tleft;
    int ht;
    int64 cycletime;

//    pthread_mutex_lock(&mutex);
//    gettimeofday(&ts, NULL);
    clock_gettime(CLOCK_MONOTONIC, &ts);  
  
    /* Conver from timeval to timespec */
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;

    while(!inOP)
    {
	osal_usleep(10000);
    }

    out_motor_t *out_motor = (out_motor_t *)ec_slave[motor1].outputs;
    in_motor_t *in_motor = (in_motor_t *)ec_slave[motor1].inputs;
	  
    while(!dorun)
    {
	osal_usleep(10000);
    }


    ec_send_processdata();

	
    while(inOP)
    {
	/* calculate next cycle start */
	add_timespec(&ts, cycletime + toff);
	/* wait to cycle start */
//	pthread_cond_timedwait(&cond, &mutex, &ts);
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
	if (dorun > 0)
	{
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
	    dorun ++;
	    /* Now running PDO */
            {

	        /* Set target position */			
		ActualPosition = (in_motor->ac_pos)/Counts_per_radian;
		ActualVelocity = (in_motor->ac_vel)/Counts_per_radian/10;

//		InputTorque = PDcontroller(ReferencePosition, ActualPosition, ActualVelocity);
		/* For damping meassure */
//		out_motor->tg_tau = (int16)(InputTorque * Units_per_Nm);
                needlf = TRUE;
	    }
	    ec_send_processdata();
	}
    }
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
    int c; uint16 BUF16;
    (void)ptr;                  /* Not used */

    do
    {
	c = getchar();
	putchar(c);
    }while (c != 'q');

    /* Disable motor drive */
    WRITE_SDO(1, 0x6040, 0, BUF16, 0, "*Control word: motor1*");
    inOP = FALSE;
    dorun = 0;
}
