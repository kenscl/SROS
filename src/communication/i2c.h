#ifndef __i2c_imu
#define __i2c_imu
#include <stdint.h>
#include "../hal/hw_specific.h"

// PB11 is SDA
// PB10 is SCL
// this uses i2c2

void configure_i2c();
void start_i2c_communication();
//returns 0 if ack, 1 if nack
void i2c_send_adress(uint8_t adress);
void i2c_send_data(uint8_t data);
void i2c_stop();
void i2c_send_data_multiple(uint8_t * data, uint32_t size);
uint8_t * i2c_recieve_data(uint8_t adress, uint32_t size);

#endif
