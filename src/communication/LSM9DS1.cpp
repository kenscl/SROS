#include "LSM9DS1.h"
#include <cstdint>
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include "../krnl/thread.h"

void LSM9DS1_write_acc_and_gyro_register(uint8_t reg, uint8_t data) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_ACC_AND_GYRO_WRITE);
    // SUB
    i2c_send_data(reg);
    // data
    i2c_send_data(data);
    // sp
    i2c_stop();
}

uint8_t * LSM9DS1_read_acc_and_gyro_register(uint8_t reg) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_ACC_AND_GYRO_WRITE);
    // SUB
    i2c_send_data(reg);
    // SR
    start_i2c_communication();
    // SAD + R + rec
    uint8_t * result = i2c_recieve_data(LSM9DS1_ACC_AND_GYRO_READ, 1);
    return result;
}


uint8_t * LSM9DS1_read_acc_and_gyro_multi(uint8_t reg, uint16_t size) {

    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_ACC_AND_GYRO_WRITE);
    // SUB
    i2c_send_data(reg);
    // SR
    start_i2c_communication();
    // SAD + R + rec
    uint8_t * result = i2c_recieve_data(LSM9DS1_ACC_AND_GYRO_READ, size);
    return result;
}

#define WRITE LSM9DS1_write_acc_and_gyro_register
#define READ LSM9DS1_read_acc_and_gyro_register

// data values
double LSM9DS1_gyro_x = 0;
double LSM9DS1_gyro_y = 0;
double LSM9DS1_gyro_z = 0;

double gyro_bias_x = 0;
double gyro_bias_y = 0;
double gyro_bias_z = 0;

void LSM9DS1_reset() {
    WRITE(CTRL_REG8, 0b1);
}

void LSM9DS1_configure_gyro() {
    // general config
    uint8_t odr = (0b110 << 5); // 952 Hz
    uint8_t fs = (0b01 << 3); // 500 dps
    uint8_t bw = (0b00 << 0); // default bw
    uint8_t ctrl_reg1_g = odr | fs | bw;
    WRITE(CTRL_REG1_G, ctrl_reg1_g);
    // enable highpass filter
    uint8_t hp = (0b1 << 6);
    uint8_t hpcf = (0b0110 << 0); // Frequenzy 1 
    uint8_t ctrl_reg3_g = hp | hpcf ;
    WRITE(CTRL_REG3_G, ctrl_reg3_g);
}

void LSM9DS1_read_gyro() {
    uint8_t * data = LSM9DS1_read_acc_and_gyro_multi(OUT_X_G_L, 6);
    LSM9DS1_gyro_x = (double) ((uint16_t)(data[1] << 8) | data[0]) / SENSITIVITY;
    LSM9DS1_gyro_y = (double) ((uint16_t)(data[3] << 8) | data[2]) / SENSITIVITY;
    LSM9DS1_gyro_z = (double) ((uint16_t)(data[5] << 8) | data[4]) / SENSITIVITY;
    LSM9DS1_gyro_x -= gyro_bias_x;
    LSM9DS1_gyro_y -= gyro_bias_y;
    LSM9DS1_gyro_z -= gyro_bias_z;
    os_free(data);
}

void LSM9DS1_thread() {
    LSM9DS1_reset();
    LSM9DS1_configure_gyro();
    // calibration
    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0;
    double tmp_x = 0;
    double tmp_y = 0;
    double tmp_z = 0;
    sleep(3 * SECONDS);
    for (int i = 0 ; i < NBR_CALIB; ++i) {
        sleep(3 * MILLISECONDS);
        LSM9DS1_read_gyro();
        tmp_x += LSM9DS1_gyro_x;
        tmp_y += LSM9DS1_gyro_y;
        tmp_z += LSM9DS1_gyro_z;
    }
    gyro_bias_x = tmp_x / NBR_CALIB;
    gyro_bias_y = tmp_y / NBR_CALIB;
    gyro_bias_z = tmp_z / NBR_CALIB;

    while (1) {
      sleep(3 * MILLISECONDS);
      LSM9DS1_read_gyro();
      os_printf("X %f, %f, Z %f \n", LSM9DS1_gyro_x, LSM9DS1_gyro_y,
                LSM9DS1_gyro_z);
    }
}
