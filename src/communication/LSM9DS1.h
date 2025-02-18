#ifndef LSM9DS1
#define LSM9DS1
#include <cstddef>
#include <cstdint>
#include <stdint.h> 
#include "i2c.h"

// devices
#define LSM9DS1_ACC_AND_GYRO_READ  0xD7
#define LSM9DS1_ACC_AND_GYRO_WRITE 0xD6
#define LSM9DS1_MAG_SENSOE_READ    0x39
#define LSM9DS1_MAG_SENSOE_WRITE   0x38

// registers
#define LSM9DS1_WHO_AM_I           0x0F
#define LSM9DS1_STATUS_REG         0x17

// gyro registers
#define REFERENCE_G                0x0B
#define CTRL_REG1_G                0x10
#define CTRL_REG3_G                0x12

#define OUT_X_G_H                  0x19
#define OUT_X_G_L                  0x18
#define OUT_Y_G_H                  0x1B
#define OUT_Y_G_L                  0x1A
#define OUT_Z_G_H                  0x1D
#define OUT_Z_G_L                  0x1C
#define CTRL_REG8                  0x22
// acc registers
#define OUT_X_XL_H                 0x29
#define OUT_X_XL_L                 0x28
#define OUT_Y_XL_H                 0x2B
#define OUT_Y_XL_L                 0x2A
#define OUT_Z_XL_H                 0x2D
#define OUT_Z_XL_L                 0x2C

// gyro constants
#define GYRO_SENSITIVITY           0.00175 // convert to dps
#define NBR_CALIB                  1000

// acc registers
#define CTRL_REG5_XL               0x1F
#define CTRL_REG6_XL               0x20

// acc constants
#define ACC_SENSITIVITY            0.000122 // convert to g
// mag

// communication
void LSM9DS1_write_acc_and_gyro_register(uint8_t reg, uint8_t data);
uint8_t * LSM9DS1_read_acc_and_gyro_register(uint8_t reg);
uint8_t * LSM9DS1_read_acc_and_gyro_multi(uint8_t reg, uint16_t size);

// configuration
void LSM9DS1_reset();
void LSM9DS1_configure_gyro();
void LSM9DS1_configure_accel();

void LSM9DS1_calibrate_sensors();

// data values
extern double LSM9DS1_gyro_x;
extern double LSM9DS1_gyro_y;
extern double LSM9DS1_gyro_z;
extern double LSM9DS1_accel_x;
extern double LSM9DS1_accel_y;
extern double LSM9DS1_accel_z;

// data read
void LSM9DS1_read_gyro();
void LSM9DS1_read_accel();

// process
void LSM9DS1_thread();

#endif 
