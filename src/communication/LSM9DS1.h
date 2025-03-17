#ifndef LSM9DS1
#define LSM9DS1
#include <cstddef>
#include <cstdint>
#include <stdint.h> 
#include "../math/vector.h"

enum SPI_state{
  Sending,
  Recieving,
  Done
};

struct SPI_information {
  SPI_state state;
  size_t size;
  uint8_t * rx_buffer;
  uint8_t * tx_buffer;
};
// use spi for communication
void SPI_init();
void CS_A_L();
void CS_A_H();
uint8_t SPI_A_transmit(uint8_t byte);
void LSM9DS1_A_write(uint8_t reg, uint8_t byte);
void LSM9DS1_A_write_dma(uint8_t reg, uint8_t byte, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer);
uint8_t LSM9DS1_A_read_register(uint8_t reg);
uint8_t LSM9DS1_A_read_register_dma(uint8_t reg, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer);
uint8_t * LSM9DS1_A_read_register_multi(uint8_t reg, uint8_t * data, size_t size);
uint8_t LSM9DS1_A_read_register_multi_dma(uint8_t reg, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer, size_t size);

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
#define ACC_SENSITIVITY            0.061 // convert to g

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

// communication

// configuration
void LSM9DS1_reset();
void LSM9DS1_configure_gyro();
void LSM9DS1_configure_accel();
void LSM9DS1_configure_mag();

void LSM9DS1_calibrate_sensors();

// data values
extern Vec3 LSM9DS1_gyro;
extern float LSM9DS1_gyro_availiable;
extern Vec3 LSM9DS1_acc;
extern float LSM9DS1_acc_availiable;
extern Vec3 LSM9DS1_mag;
extern float LSM9DS1_mag_availiable;

// data read
void LSM9DS1_read_status();
void LSM9DS1_read_gyro();
void LSM9DS1_read_accel();
void LSM9DS1_read_mag();

// check i2c status
uint8_t LSM9DS1_check_status();
uint8_t LSM9DS1_check_gyro();
uint8_t LSM9DS1_check_accel();
uint8_t LSM9DS1_check_mag();

// enable send
void LSM9DS1_enable_status();
void LSM9DS1_enable_gyro();
void LSM9DS1_enable_accel();
void LSM9DS1_enable_mag();

// process
void  LSM9DS1_process_status();
void  LSM9DS1_process_gyro();
void  LSM9DS1_process_accel();
volatile void LSM9DS1_thread();

#endif 
