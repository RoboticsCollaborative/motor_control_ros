#ifndef INIT_BEL_H
#define INIT_BEL_H

#include <osal.h>

int SDO_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value);
int SDO_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
int SDO_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);

int map_BEL_CSP_PDOs(uint16 slave);
int map_BEL_CSP_callback(uint16 motor);

int init_BEL1_CSP(uint16 motor);
int init_BEL2_CSP(uint16 motor);


#endif /* INIT_BEL_H */
