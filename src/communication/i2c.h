#ifndef __i2c_imu
#define __i2c_imu
#include <cstdint>
#include <stdint.h>
#include "../hal/hw_specific.h"

enum I2C1_state_enum {
    Idle,
    Busy, 
    Error
};

extern I2C1_state_enum I2C1_state;

enum I2C_state_comm {
    Sending,
    Recieving,
    Done
};

struct I2C_state_information {
    I2C_state_comm state;
    uint32_t device_adress_write;
    uint32_t device_adress_recieve;
    uint32_t device_subadress;
    uint32_t recieve_bytes;
    uint8_t * data;
};


// PB11 is SDA
// PB10 is SCL
// this uses i2c1

void configure_i2c();
void start_i2c_communication();
//returns 0 if ack, 1 if nack
void i2c_send_adress(uint8_t adress);
void i2c_send_data(uint8_t data);
void i2c_stop();
void i2c_send_data_multiple(uint8_t * data, uint32_t size);
void i2c_recieve_single();
// void i2c_recieve_multi(uint8_t adress, uint32_t size); // handled explicitly in interrup handler 

uint8_t i2c_handle(I2C_state_information * info);
volatile void i2c_thread();

#endif
