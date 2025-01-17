#include "LSM9DS1.h"

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
