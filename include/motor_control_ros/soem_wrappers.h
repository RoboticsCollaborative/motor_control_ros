#ifndef SOEM_WRAPPERS_H
#define SOEM_WRAPPERS_H

/* Buffers for SDO transfer */
#define READ_SDO(slave, idx, sub, buf, comment) \
{ \
    buf=0; \
    int __s = sizeof(buf); \
    int __ret = ec_SDOread(slave, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM); \
    printf("Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
}

#define WRITE_SDO(slave, idx, sub, buf, value, comment) \
{ \
    int __s = sizeof(buf); \
    buf = value; \
    int __ret = ec_SDOwrite(slave, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM); \
    printf("Write at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
}

#define CHECKERROR(slave) \
{ \
    ec_readstate(); \
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slave].state, ec_slave[slave].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slave].ALstatuscode)); \
}

#endif /* SOEM_WRAPPERS_H */
