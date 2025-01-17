#ifndef LSM9DS1
#define LSM9DS1
#include <stdint.h> 
#include "i2c.h"

// devices
#define LSM9DS1_ACC_AND_GYRO_READ  0xD7
#define LSM9DS1_ACC_AND_GYRO_WRITE 0xD6
#define LSM9DS1_MAG_SENSOE_READ    0x39
#define LSM9DS1_MAG_SENSOE_WRITE   0x38

// registers
#define LSM9DS1_WHO_AM_I           0x0F

//void LSM9DS1_write_register();
uint8_t * LSM9DS1_read_acc_and_gyro_register(uint8_t reg);

#endif 
