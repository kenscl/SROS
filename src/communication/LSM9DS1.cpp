#include "LSM9DS1.h"
#include <cstdint>
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include "../krnl/thread.h"
#include "../krnl/scheduler.h"

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

double LSM9DS1_acc_x = 0;
double LSM9DS1_acc_y = 0;
double LSM9DS1_acc_z = 0;

volatile double acc_bias_x = 0;
volatile double acc_bias_y = 0;
volatile double acc_bias_z = 0;

volatile double gyro_bias_x = 0;
volatile double gyro_bias_y = 0;
volatile double gyro_bias_z = 0;


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


void LSM9DS1_configure_accel() {
    // general config
    uint8_t odr = (0b110 << 5); // 952 Hz
    uint8_t fs = (0b01 << 3); // 4 g
    uint8_t bw_scale = (0b0 << 2); // bw according to odr
    uint8_t bw = (0b00 << 0); // default bw, dosnt matter
    uint8_t ctrl_reg6_xl = odr | fs | bw;
    WRITE(CTRL_REG6_XL, ctrl_reg6_xl);
    
}

void LSM9DS1_calibrate_sensors() {
    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0;
    acc_bias_x = acc_bias_y = acc_bias_z = 0;

    double tmp_x = 0;
    double tmp_y = 0;
    double tmp_z = 0;
    double tmp_x_acc = 0;
    double tmp_y_acc = 0;
    double tmp_z_acc = 0;

    sleep(3 * SECONDS);
    for (int i = 0 ; i < NBR_CALIB; ++i) {
        LSM9DS1_read_gyro();
        LSM9DS1_read_accel();

        tmp_x += LSM9DS1_gyro_x;
        tmp_y += LSM9DS1_gyro_y;
        tmp_z += LSM9DS1_gyro_z;

        tmp_x_acc += LSM9DS1_acc_x;
        tmp_y_acc += LSM9DS1_acc_y;
        tmp_z_acc += LSM9DS1_acc_z;
        sleep(3 * MILLISECONDS);
    }
    gyro_bias_x = tmp_x / NBR_CALIB;
    gyro_bias_y = tmp_y / NBR_CALIB;
    gyro_bias_z = tmp_z / NBR_CALIB;
    acc_bias_x = tmp_x_acc / NBR_CALIB;
    acc_bias_y = tmp_y_acc / NBR_CALIB;
    acc_bias_z = tmp_z_acc / NBR_CALIB;


}

void LSM9DS1_read_gyro() {
    uint8_t * data = LSM9DS1_read_acc_and_gyro_multi(OUT_X_G_L, 6);
    LSM9DS1_gyro_x = (double) ((uint16_t)(data[1] << 8) | data[0]) * GYRO_SENSITIVITY;
    LSM9DS1_gyro_y = (double) ((uint16_t)(data[3] << 8) | data[2]) * GYRO_SENSITIVITY;
    LSM9DS1_gyro_z = (double) ((uint16_t)(data[5] << 8) | data[4]) * GYRO_SENSITIVITY;
    LSM9DS1_gyro_x -= gyro_bias_x;
    LSM9DS1_gyro_y -= gyro_bias_y;
    LSM9DS1_gyro_z -= gyro_bias_z;
    os_free(data);
}

void LSM9DS1_read_accel() {
    uint8_t * data = LSM9DS1_read_acc_and_gyro_multi(OUT_X_XL_L, 6);
    LSM9DS1_acc_x = (double) ((uint16_t)(data[1] << 8) | data[0]) * ACC_SENSITIVITY;
    LSM9DS1_acc_y = (double) ((uint16_t)(data[3] << 8) | data[2]) * ACC_SENSITIVITY;
    LSM9DS1_acc_z = (double) ((uint16_t)(data[5] << 8) | data[4]) * ACC_SENSITIVITY;
    LSM9DS1_acc_x -= acc_bias_x;
    LSM9DS1_acc_y -= acc_bias_y;
    LSM9DS1_acc_z -= acc_bias_z;
    os_free(data);
}

void LSM9DS1_thread() {
    LSM9DS1_reset();
    LSM9DS1_configure_gyro();
    LSM9DS1_configure_accel();
    // calibration
    LSM9DS1_calibrate_sensors();

    while (1) {
        LSM9DS1_read_gyro();
        LSM9DS1_read_accel();
      //os_printf("%d, %f, %f, %f\n", (int)now(), LSM9DS1_gyro_x, LSM9DS1_gyro_y,
      //          LSM9DS1_gyro_z);
        os_printf("%d, %f, %f, %f\n", (int)now(), LSM9DS1_acc_x, LSM9DS1_acc_y,
                  LSM9DS1_acc_z);
        os_printf("%d, %f, %f, %f\n", (int)now(), acc_bias_x, gyro_bias_x,
                  LSM9DS1_acc_z);
        sleep(1 * SECONDS);
    }
}
