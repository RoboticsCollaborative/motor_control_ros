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
#include <inttypes.h>

#include <soem/ethercat.h>
//#include "motor_control_ros/ethercat.h"
#include "motor_control_ros/ecattype.h"

/** SDO transfer functions 
 *
 * @param[in] slave	=  Slave number.
 * @param[in] index	=  Index to write.
 * @param[in] subindex	=  Subindex to write, must be 0 or 1 if CA is used.	
 * @param[in] value	=  Value to write.
 * @return wkc after a single SDO is trasfered.
 */

int SDO_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

int SDO_write16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

int SDO_write32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}


/** Selective PDO mapping
 *
 * @param[in] slave	=  Slave number.
 * @param[out] wkc 	=  Working counter.
 * */
int motor_pdo (uint16 slave)
{
    int wkc = 0;
    
    printf ("Motor drive setup\n");

    wkc += SDO_write8  (slave, 0x1C12, 0, 0);
    wkc += SDO_write8  (slave, 0x1C13, 0, 0);

    /* CSP Inputs */
/*
    wkc += SDO_write8  (slave, 0x1A00, 0, 0);
    wkc += SDO_write32 (slave, 0x1A00, 1, 0x60640020); //Actual position
    wkc += SDO_write32 (slave, 0x1A00, 2, 0x606C0020); //Actual velocity
    wkc += SDO_write8  (slave, 0x1A00, 0, 2);
*/

    /* CSP Outputs */
/*
    wkc += SDO_write8  (slave, 0x1600, 0, 0);
    wkc += SDO_write32 (slave, 0x1600, 1, 0x607A0020); //Target position
    wkc += SDO_write32 (slave, 0x1600, 2, 0x60710010); //Target torque
    wkc += SDO_write8  (slave, 0x1600, 0, 2);
*/

    /* CSP Outputs */
    wkc += SDO_write16 (slave, 0x1C12, 1, 0X1700);
    wkc += SDO_write8  (slave, 0x1C12, 0, 1);

    /* CSP Inputs */
    wkc += SDO_write16 (slave, 0x1C13, 1, 0x1B00);
    wkc += SDO_write8  (slave, 0x1C13, 0, 1);


    /* Explicitly set flags that are (probably) invalid in EEPROM */
    ec_slave[slave].SM[2].SMflags = 0x10024l;

    /* Explicitly disable the sync manager that are activated by EEPROM */
    ec_slave[slave].SM[4].StartAddr = 0;
    ec_slave[slave].SM[5].StartAddr = 0;

    return wkc;
}


/** PDO mapping for motor
 *
 * @param[in] slave	=  Slave number.
 * @return -1 if motor setup fails.
 */
int motor_setup(uint16 motor)
{
    ec_slave[motor].PO2SOconfig = motor_pdo;
}


/** Initialize motor parameters
 *
 * @param[in] motor	=  Slave number.
 * @return -1 if motor setup fails.
 */
int motor_init(uint16 motor)
{

    uint8 BUF8; uint32 BUF32;

    /* Switch to torque mode */
    /* OpMode: 8 => CSP mode */
    /* OpMode: 10 => Torque mode */
    WRITE_SDO(motor, 0x6060, 0, BUF8, 8, "OpMode");
    READ_SDO(motor, 0x6061, 0, BUF8, "OpMode display");
    /* rated torque (0.001 Nm) */
    /* 1 Nm = 5000 units (up to 4Nm) */
    /* 1 unit = 0.2 mNm */ 
    WRITE_SDO(motor, 0x6076, 0, BUF32, 200, "Motor rated torque");

    return 0;
}


