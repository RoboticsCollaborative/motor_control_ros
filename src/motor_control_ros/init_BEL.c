/**
 * \file
 * \ Configure motor static parameters.
 * 
 * Edited by Zikun Yu -- yu.zik@husky.neu.edu
 *
 * Whitney Robotics Lab  10/2/2018
 */

#include <stdio.h>
#include <stdlib.h>
//#include <inttypes.h>
#include <soem/ethercat.h>

#include "motor_control_ros/init_BEL.h"
#include "motor_control_ros/soem_wrappers.h"


int SDO_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value) {
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

int SDO_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value) {
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

int SDO_write32 (uint16 slave, uint16 index, uint8 subindex, uint32 value) {
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}


int map_BEL_CSP_PDOs (uint16 slave) {
    
    int wkc = 0;
    
    /* We clear SM2 (0x1C12:0) and SM3 (0x1C13:0) before they are configured,
     * and then list the PDO in subindices 1,2,etc. and then finally set the
     * number of PDOs mapped on subindex 0, which we previously cleared.
     * These mapping are documented in the slave ESI file.
     * 
     * BEL CSP RxPDOs (master outputs) */
    wkc += SDO_write8  (slave, 0x1C12, 0, 0); /* clear SM2 (slave RxPDOs) */
    wkc += SDO_write16 (slave, 0x1C12, 1, 0x1700); /* pre-mapped PDO */
    wkc += SDO_write8  (slave, 0x1C12, 0, 1); /* set # of mapped PDOs */


    /* BEL CSP TxPDOs (master inputs) */
    
    /* Use user-defined PDO to map the load encoder position and velocity, you
     * need to reference documentation on PDO mapping objects to understand the
     * values set here.  In a nutshell:  {object index,sub-index,size in bits}
     * This mapping defines user TxPDOs (inputs to master)
     */
    wkc += SDO_write8  (slave, 0x1A00, 0, 0); /* clear the PDO first */
    wkc += SDO_write32 (slave, 0x1A00, 2, 0x22420020); /* Load encoder position */
    wkc += SDO_write32 (slave, 0x1A00, 1, 0x22310020); /* Load encoder velocity */
    wkc += SDO_write8  (slave, 0x1A00, 0, 2); /* set number of objects mapped by PDO */

    /* pre-mapped PDOs that the slave sends to the master */
    wkc += SDO_write8  (slave, 0x1C13, 0, 0); /* clear SM3 (slave TxPDOs) */
    wkc += SDO_write16 (slave, 0x1C13, 1, 0x1B00); /* pre-mapped PDO */
    wkc += SDO_write16 (slave, 0x1C13, 2, 0x1A00); /* user-PDO */
    wkc += SDO_write8  (slave, 0x1C13, 0, 2); /* set # of mapped PDOs */


    /* as specified in ESI file, set control word during PRE->SAFE transition */
    SDO_write16(slave, 0x6060, 0, 8);
    
    return wkc;
}


/* Attach a callback function for PRE->SAFE transition
 */
int map_BEL_CSP_callback(uint16 motor) {
    ec_slave[motor].PO2SOconfig = map_BEL_CSP_PDOs;
    return 0;
}


/* Initialize BEL/motor parameters via SDO
 */
int init_BEL1_CSP(uint16 motor) {
    
    printf ("Motor drive %d init\n", motor);

    uint16 BUF16; 
    int16 IBUF16; 
    int32 IBUF32; 
    //uint8 BUF8; 
    uint32 BUF32;

    /* Motor params */
    WRITE_SDO(motor, 0x2383, 12, IBUF32, 25456, "Motor torque constant");
    WRITE_SDO(motor, 0x2383, 13, IBUF32, 650000, "Motor peak torque"); 
    WRITE_SDO(motor, 0x2383, 14, IBUF32, 20000, "Motor continuous torque"); 

    //~ READ_SDO(motor, 0x2383, 22, BUF16, "Motor encoder direction");
    //~ osal_usleep(1000);
    //~ WRITE_SDO(motor, 0x2383, 22, BUF16, 1, "Reverse motor direction"); // continuous torque = 0.2 Nm 
    //~ osal_usleep(1000);
    //~ READ_SDO(motor, 0x2383, 22, BUF16, "Motor encoder direction");
    //~ osal_usleep(1000);
    //~ READ_SDO(motor, 0x2383, 3, BUF16, "Motor wiring configuration");
    //~ osal_usleep(1000);


    /* Loop gains */
    WRITE_SDO(motor, 0x2382, 1, BUF16, 0, "Position loop gain (Pp)");
    WRITE_SDO(motor, 0x2381, 1, BUF16, 0, "Velocity loop gain (Vp)");
    //READ_SDO (motor, 0x2382, 1, BUF16, "Velocity loop gain (Pp)");
    //READ_SDO (motor, 0x2381, 1, BUF16, "Velocity loop gain (Vp)");

    /* Motor limits */
    WRITE_SDO(motor, 0x2110, 0,  IBUF16, 1400, "Peak current limit");
    WRITE_SDO(motor, 0x2111, 0,  IBUF16, 700, "Continuous current limit"); /* units of 0.01A */
    //READ_SDO(motor, 0x2111, 0, IBUF16, "Continuous current limit");
    //READ_SDO(motor, 0x2110, 0, IBUF16, "Peak current limit");
    //READ_SDO(motor, 0x2103, 0, IBUF32, "Velocity limit");


    //READ_SDO(motor, 0x6061, 0, BUF8, "OpMode display");
    
    /* rated torque (0.001 Nm) */
    /* 1 Nm = 5000 units (up to 4Nm) */
    /* 1 unit = 0.2 mNm, CoE */ 
    WRITE_SDO(motor, 0x6076, 0, BUF32, 200, "Motor rated torque");

    return 0;
}

int init_BEL2_CSP(uint16 motor) {
    
    printf ("Motor drive %d init\n", motor);

    uint16 BUF16; 
    int16 IBUF16; 
    int32 IBUF32; 
    //uint8 BUF8; 
    uint32 BUF32;

    /* Motor params */
    WRITE_SDO(motor, 0x2383, 12, IBUF32, 25456, "Motor torque constant");
    WRITE_SDO(motor, 0x2383, 13, IBUF32, 650000, "Motor peak torque"); 
    WRITE_SDO(motor, 0x2383, 14, IBUF32, 20000, "Motor continuous torque"); 

    //~ READ_SDO(motor, 0x2383, 22, BUF16, "Motor encoder direction");
    //~ osal_usleep(1000);
    //~ WRITE_SDO(motor, 0x2383, 22, BUF16, 1, "Reverse motor direction"); // continuous torque = 0.2 Nm 
    //~ osal_usleep(1000);
    //~ READ_SDO(motor, 0x2383, 22, BUF16, "Motor encoder direction");
    //~ osal_usleep(1000);
    //~ READ_SDO(motor, 0x2383, 3, BUF16, "Motor wiring configuration");
    //~ osal_usleep(1000);


    /* Loop gains */
    WRITE_SDO(motor, 0x2382, 1, BUF16, 0, "Position loop gain (Pp)");
    WRITE_SDO(motor, 0x2381, 1, BUF16, 0, "Velocity loop gain (Vp)");
    //READ_SDO (motor, 0x2382, 1, BUF16, "Velocity loop gain (Pp)");
    //READ_SDO (motor, 0x2381, 1, BUF16, "Velocity loop gain (Vp)");

    /* Motor limits */
    WRITE_SDO(motor, 0x2110, 0,  IBUF16, 1400, "Peak current limit");
    WRITE_SDO(motor, 0x2111, 0,  IBUF16, 700, "Continuous current limit"); /* units of 0.01A */
    //READ_SDO(motor, 0x2111, 0, IBUF16, "Continuous current limit");
    //READ_SDO(motor, 0x2110, 0, IBUF16, "Peak current limit");
    //READ_SDO(motor, 0x2103, 0, IBUF32, "Velocity limit");


    //READ_SDO(motor, 0x6061, 0, BUF8, "OpMode display");
    
    /* rated torque (0.001 Nm) */
    /* 1 Nm = 5000 units (up to 4Nm) */
    /* 1 unit = 0.2 mNm, CoE */ 
    WRITE_SDO(motor, 0x6076, 0, BUF32, 200, "Motor rated torque");

    return 0;
}


