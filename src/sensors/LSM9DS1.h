#ifndef LSM9DS1
#define LSM9DS1
#include <stddef.h>
#include <stdint.h>
#include <stdint.h>
#include "../communication/SPI.h"
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
extern void CS_A_H(void);
extern void CS_A_L(void);
extern void CS_M_H(void);
extern void CS_M_L(void);

void setup_cs_lines();

// configuration
void LSM9DS1_reset(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG1_G(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG3_G(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG6_XL(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG1_M(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG2_M(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG3_M(SPI_INFO *fill);
void LSM9DS1_WRITE_CTRL_REG4_M(SPI_INFO *fill);

// reading
void LSM9DS1_read_WHO_AM_I_A(SPI_INFO *fill);
void LSM9DS1_read_WHO_AM_I_M(SPI_INFO *fill);
void LSM9DS1_read_gyro(SPI_INFO *fill);
void LSM9DS1_read_acc(SPI_INFO *fill);
void LSM9DS1_read_mag(SPI_INFO *fill);

void LSM9DS1_READ_CTRL_REG4_M(SPI_INFO *fill);

void LSM9DS1_calibrate_sensors();

// data values
extern Vec3 LSM9DS1_gyro;
extern Vec3 LSM9DS1_gyro_filtered;
extern float LSM9DS1_gyro_availiable;
extern Vec3 LSM9DS1_acc;
extern Vec3 LSM9DS1_acc_filtered;
extern float LSM9DS1_acc_availiable;
extern Vec3 LSM9DS1_mag;
extern Vec3 LSM9DS1_mag_filtered;
extern float LSM9DS1_mag_availiable;

// process
void low_pass_filter(float alpha, Vec3 *mean, Vec3 *new_measurement);
void LSM9DS1_process_status();
void LSM9DS1_process_gyro();
void LSM9DS1_process_accel();
void LSM9DS1_process_WHO_AM_I();
volatile void LSM9DS1_thread();

#endif
