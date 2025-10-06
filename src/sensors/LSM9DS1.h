#ifndef LSM9DS1
#define LSM9DS1
#include <stddef.h>
#include <stdint.h>
#include <stdint.h>
#include "../math/vector.h"
#include "../math/matrix.h"


// devices
#define LSM9DS1_ACC_AND_GYRO_READ  0xD7
#define LSM9DS1_ACC_AND_GYRO_WRITE 0xD6
#define LSM9DS1_MAG_SENSOE_READ    0x3D
#define LSM9DS1_MAG_SENSOE_WRITE   0x3C

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
#define GYRO_SENSITIVITY           17.5 // convert to dps
#define NBR_CALIB                  100

// acc registers
#define CTRL_REG5_XL               0x1F
#define CTRL_REG6_XL               0x20
#define CTRL_REG7_XL               0x21

// acc constants
#define ACC_SENSITIVITY            0.122 // convert to g

// mag registers
#define LSM9DS1_STATUS_REG_M         0x27
#define CTRL_REG1_M               0x20
#define CTRL_REG2_M               0x21
#define CTRL_REG3_M               0x22
#define CTRL_REG4_M               0x23

#define OUT_X_H_M                 0x29
#define OUT_X_L_M                 0x28
#define OUT_Y_H_M                 0x2B
#define OUT_Y_L_M                 0x2A
#define OUT_Z_H_M                 0x2D
#define OUT_Z_L_M                 0x2C

// mag constants
#define MAG_SENSITIVITY           0.14

// read write constants

#define LSM9DS1_WRITE_REGISTER(reg)   ((reg) & 0x7F)
#define LSM9DS1_READ_REGISTER(reg)    ((reg) | 0x80)

// cs-lines
void CS_A_H();

void CS_A_L();

void CS_M_H();

void CS_M_L();

// configuration
void LSM9DS1_reset();
void LSM9DS1_configure_gyro();
void LSM9DS1_configure_accel();
void LSM9DS1_configure_mag();

void LSM9DS1_calibrate_sensors();

// data values
extern Vec *LSM9DS1_gyro;
extern Vec *LSM9DS1_gyro_filtered;
extern float LSM9DS1_gyro_availiable;
extern Vec *LSM9DS1_acc;
extern Vec *LSM9DS1_acc_filtered;
extern float LSM9DS1_acc_availiable;
extern Vec *LSM9DS1_mag;
extern Vec *LSM9DS1_mag_filtered;
extern float LSM9DS1_mag_availiable;

// data read
void LSM9DS1_read_status();
void LSM9DS1_read_gyro();
void LSM9DS1_read_accel();
void LSM9DS1_read_mag();
void LSM9DS1_read_WHO_AM_I();

// check i2c status
uint8_t LSM9DS1_check_status();
uint8_t LSM9DS1_check_gyro();
uint8_t LSM9DS1_check_accel();
uint8_t LSM9DS1_check_mag();
uint8_t LSM9DS1_check_WHO_AM_I();

// enable send
void LSM9DS1_enable_status();
void LSM9DS1_enable_gyro();
void LSM9DS1_enable_accel();
void LSM9DS1_enable_mag();
void LSM9DS1_enable_WHO_AM_I();

// process
Vec low_pass_filter(float alpha, Vec mean, Vec new_measurement);
void LSM9DS1_process_status();
void LSM9DS1_process_gyro();
void LSM9DS1_process_accel();
void LSM9DS1_process_WHO_AM_I();
volatile void LSM9DS1_thread();

#endif
